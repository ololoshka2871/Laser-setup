#![no_main]
#![no_std]
#![feature(macro_metavar_expr)]
#![feature(alloc_error_handler)]

mod config;
mod hw;
mod protobuf;
mod protobuf_server;
mod shift;
mod support;

use defmt_rtt as _; // global logger
use panic_probe as _;

extern crate alloc;

use umm_malloc;

use rtic::app;

use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::{
    Alternate, GpioExt, IOPinSpeed, OpenDrain, Output, OutputSpeed, PinState, PushPull, PA5, PA6,
    PA7, PB10, PB11, PB6, PB7,
};
use stm32f1xx_hal::pac::Interrupt;
use stm32f1xx_hal::pac::{I2C1, SPI1};
use stm32f1xx_hal::rcc::{HPre, PPre};
use stm32f1xx_hal::spi::{NoMiso, Spi, Spi1NoRemap};
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};

use usb_device::prelude::{UsbDevice, UsbDeviceBuilder};

use usbd_serial::SerialPort;

use systick_monotonic::Systick;

use i2c_hung_fix::HangFixPins;

use support::clocking::{ClockConfigProvider, MyConfig};

use hw::{ADC_DEVIDER, AHB_DEVIDER, APB1_DEVIDER, APB2_DEVIDER, PLL_MUL, PLL_P_DIV, USB_DEVIDER};

//-----------------------------------------------------------------------------

struct HighPerformanceClockConfigProvider;

impl HighPerformanceClockConfigProvider {
    fn ahb_dev2val(ahb_dev: HPre) -> u32 {
        match ahb_dev {
            HPre::DIV1 => 1,
            HPre::DIV2 => 2,
            HPre::DIV4 => 4,
            HPre::DIV8 => 8,
            HPre::DIV16 => 16,
            HPre::DIV64 => 64,
            HPre::DIV128 => 128,
            HPre::DIV256 => 256,
            HPre::DIV512 => 512,
        }
    }

    fn apb_dev2val(apb_dev: PPre) -> u32 {
        match apb_dev {
            PPre::DIV1 => 1,
            PPre::DIV2 => 2,
            PPre::DIV4 => 4,
            PPre::DIV8 => 8,
            PPre::DIV16 => 16,
        }
    }

    fn pll_mul_bits(mul: u32) -> u8 {
        (mul - 2) as u8
    }

    fn ppl_div2val(div: stm32f1xx_hal::device::rcc::cfgr::PLLXTPRE_A) -> u32 {
        match div {
            stm32f1xx_hal::device::rcc::cfgr::PLLXTPRE_A::DIV1 => 1,
            stm32f1xx_hal::device::rcc::cfgr::PLLXTPRE_A::DIV2 => 2,
        }
    }

    fn freeze(_acr: &mut stm32f1xx_hal::flash::ACR) -> stm32f1xx_hal::rcc::Clocks {
        use stm32f1xx_hal::time::MHz;

        let cfg = Self::to_config();

        let clocks = cfg.get_clocks();
        // adjust flash wait states
        let acr = unsafe { &*stm32f1xx_hal::device::FLASH::ptr() };
        unsafe {
            acr.acr.write(|w| {
                w.latency().bits(if clocks.sysclk() <= MHz(24) {
                    0b000
                } else if clocks.sysclk() <= MHz(48) {
                    0b001
                } else {
                    0b010
                })
            })
        }

        let rcc = unsafe { &*stm32f1xx_hal::device::RCC::ptr() };

        if cfg.hse.is_some() {
            // enable HSE and wait for it to be ready

            rcc.cr.modify(|_, w| w.hseon().set_bit());

            while rcc.cr.read().hserdy().bit_is_clear() {}
        }

        if let Some(pllmul_bits) = cfg.pllmul {
            // enable PLL and wait for it to be ready

            #[allow(unused_unsafe)]
            rcc.cfgr
                .modify(|_, w| unsafe { w.pllxtpre().variant(PLL_P_DIV) });

            #[allow(unused_unsafe)]
            rcc.cfgr.modify(|_, w| unsafe {
                w.pllmul().bits(pllmul_bits).pllsrc().bit(cfg.hse.is_some())
            });

            rcc.cr.modify(|_, w| w.pllon().set_bit());

            while rcc.cr.read().pllrdy().bit_is_clear() {}
        }

        rcc.cfgr.modify(|_, w| unsafe {
            w.adcpre().variant(cfg.adcpre);
            w.ppre2()
                .bits(cfg.ppre2 as u8)
                .ppre1()
                .bits(cfg.ppre1 as u8)
                .hpre()
                .bits(cfg.hpre as u8)
                .usbpre()
                .variant(cfg.usbpre)
                .sw()
                .bits(if cfg.pllmul.is_some() {
                    // PLL
                    0b10
                } else if cfg.hse.is_some() {
                    // HSE
                    0b1
                } else {
                    // HSI
                    0b0
                })
        });

        clocks
    }
}

impl ClockConfigProvider for HighPerformanceClockConfigProvider {
    fn core_frequency() -> Hertz {
        let f = crate::config::XTAL_FREQ / Self::ppl_div2val(PLL_P_DIV) * PLL_MUL
            / Self::ahb_dev2val(AHB_DEVIDER);
        Hertz::Hz(f)
    }

    fn apb1_frequency() -> Hertz {
        Hertz::Hz(Self::core_frequency().to_Hz() / Self::apb_dev2val(APB1_DEVIDER))
    }

    fn apb2_frequency() -> Hertz {
        Hertz::Hz(Self::core_frequency().to_Hz() / Self::apb_dev2val(APB2_DEVIDER))
    }

    // stm32_cube: if APB devider > 1, timers freq APB*2
    fn master_counter_frequency() -> Hertz {
        // TIM3 - APB1
        if APB2_DEVIDER == PPre::DIV1 {
            Self::core_frequency()
        } else {
            Self::core_frequency() * 2
        }
    }

    fn xtal2master_freq_multiplier() -> f32 {
        PLL_MUL as f32
            / (Self::ppl_div2val(PLL_P_DIV)
                * Self::ahb_dev2val(AHB_DEVIDER)
                * Self::apb_dev2val(APB2_DEVIDER)) as f32
    }

    fn to_config() -> MyConfig {
        MyConfig {
            hse_p_div: PLL_P_DIV,
            hse: Some(crate::config::XTAL_FREQ),
            pllmul: Some(Self::pll_mul_bits(PLL_MUL)),
            hpre: AHB_DEVIDER,
            ppre1: APB1_DEVIDER,
            ppre2: APB2_DEVIDER,
            usbpre: USB_DEVIDER,
            adcpre: ADC_DEVIDER,
        }
    }
}

//-----------------------------------------------------------------------------

#[alloc_error_handler]
fn oom(oom_layout: core::alloc::Layout) -> ! {
    defmt::panic!("Out of memory: {:?}", defmt::Debug2Format(&oom_layout));
}

//-----------------------------------------------------------------------------

static mut HEAP: [u8; config::HEAP_SIZE] = [0; config::HEAP_SIZE];

type TShifter = shift::Shifter<
    Spi<SPI1, Spi1NoRemap, (PA5<Alternate<PushPull>>, NoMiso, PA7<Alternate<PushPull>>), u8>,
    PA6<Output<PushPull>>,
    1,
>;

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM])]
mod app {
    use super::*;

    use embedded_hal::blocking::spi::Write;

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBus<Peripheral>>,
    }

    #[local]
    struct Local {
        valve: PB10<Output<PushPull>>,
        actuator: PB11<Output<PushPull>>,

        shifter: TShifter,

        sr: shift::ShifterRef,
        i2c1: stm32f1xx_hal::i2c::BlockingI2c<
            I2C1,
            (PB6<Alternate<OpenDrain>>, PB7<Alternate<OpenDrain>>),
        >,

        my_delay: MyDelay,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<{ config::SYSTICK_RATE_HZ }>;

    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        if cfg!(debug_assertions) {
            // need for defmt logging works https://github.com/knurling-rs/probe-run/pull/183/files
            ctx.device.RCC.ahbenr.modify(|_, w| w.dma1en().enabled());
        }

        defmt::info!("Init...");

        ctx.core.DCB.enable_trace();
        ctx.core.DWT.enable_cycle_counter();
        defmt::info!("DWT...");

        let mut flash = ctx.device.FLASH.constrain();

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();

        let mut afio = ctx.device.AFIO.constrain();

        let actuator = gpiob
            .pb11
            .into_push_pull_output_with_state(&mut gpiob.crh, config::ACTUATOR_CLOSE_STATE);

        let valve = gpiob
            .pb10
            .into_push_pull_output_with_state(&mut gpiob.crh, config::VALVE_ATMOSPHERE_STATE);

        let mut usb_pull_up = gpiob
            .pb8
            .into_push_pull_output_with_state(&mut gpiob.crh, config::USB_PULLUP_ACTVE_LEVEL);

        let (mut mosi, mut sck, mut lat) = (
            gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl),
            gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl),
            gpioa
                .pa6
                .into_push_pull_output_with_state(&mut gpioa.crl, PinState::Low),
        );
        mosi.set_speed(&mut gpioa.crl, IOPinSpeed::Mhz2);
        sck.set_speed(&mut gpioa.crl, IOPinSpeed::Mhz2);
        lat.set_speed(&mut gpioa.crl, IOPinSpeed::Mhz2);

        defmt::info!("\tPins");

        let clocks = HighPerformanceClockConfigProvider::freeze(&mut flash.acr);

        let mut my_delay = MyDelay {
            sys_clk: clocks.sysclk(),
        };

        defmt::info!("\tClocks: {}", defmt::Debug2Format(&clocks));

        unsafe { umm_malloc::init_heap(HEAP.as_mut_ptr() as usize, HEAP.len()) };

        defmt::info!("\tHeap");

        let mono = Systick::new(ctx.core.SYST, clocks.sysclk().to_Hz());

        let usb = Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };

        let mut spi1 = stm32f1xx_hal::spi::Spi::spi1(
            ctx.device.SPI1,
            (sck, NoMiso, mosi),
            &mut afio.mapr,
            embedded_hal::spi::MODE_0,
            Hertz::kHz(100),
            clocks,
        );

        spi1.write(&[0x00, 0x00]).ok();

        let mut shifter = shift::Shifter::<_, _, 1>::new(spi1, lat);
        let sr0 = shifter.add(16);

        defmt::info!("\tSPI");

        let mut i2c_pins = (
            gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
            gpiob.pb7.into_floating_input(&mut gpiob.crl),
        );

        i2c_pins
            .try_unhang_i2c(&mut my_delay)
            .expect("I2C unhang failed");
        let i2c_pins = (
            i2c_pins.0.into_alternate_open_drain(&mut gpiob.crl),
            i2c_pins.1.into_alternate_open_drain(&mut gpiob.crl),
        );
        let i2c1 = stm32f1xx_hal::i2c::BlockingI2c::i2c1(
            ctx.device.I2C1,
            i2c_pins,
            &mut afio.mapr,
            stm32f1xx_hal::i2c::Mode::Standard {
                frequency: Hertz::kHz(400),
            },
            clocks,
            20000,
            1,
            20000,
            20000, // должен успеть проходить SCL stretch
        );

        defmt::info!("\tI2C");

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }

        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap_unchecked() });

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            usb_device::prelude::UsbVidPid(0x0483, 0x5720),
        )
        .manufacturer("SCTBElpa")
        .product("laser-setup")
        .serial_number("1")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        defmt::info!("\tUSB");

        //---------------------------------------------------------------------

        usb_pull_up.toggle(); // enable USB
        defmt::info!("\tUSB enabled");

        //---------------------------------------------------------------------

        (
            Shared {
                usb_device: usb_dev,
                serial,
            },
            Local {
                actuator,
                valve,
                shifter,
                sr: sr0,
                i2c1,
                my_delay,
            },
            init::Monotonics(mono),
        )
    }

    //-------------------------------------------------------------------------

    #[task(binds = USB_HP_CAN_TX, shared = [usb_device, serial], priority = 1)]
    fn usb_tx(ctx: usb_tx::Context) {
        let mut usb_device = ctx.shared.usb_device;
        let mut serial = ctx.shared.serial;

        if (&mut usb_device, &mut serial).lock(|usb_device, serial| usb_device.poll(&mut [serial]))
        {
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_HP_CAN_TX);
        }
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_device, serial], priority = 1)]
    fn usb_rx0(ctx: usb_rx0::Context) {
        let mut usb_device = ctx.shared.usb_device;
        let mut serial = ctx.shared.serial;

        if (&mut usb_device, &mut serial).lock(|usb_device, serial| usb_device.poll(&mut [serial]))
        {
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_LP_CAN_RX0);
        }
    }

    //-------------------------------------------------------------------------

    #[idle(shared=[serial], local = [actuator, valve, shifter, sr, i2c1, my_delay])]
    fn idle(ctx: idle::Context) -> ! {
        let mut serial = ctx.shared.serial;
        let actuator = ctx.local.actuator;
        let valve = ctx.local.valve;
        let shifter = ctx.local.shifter;
        let sr = *ctx.local.sr;
        let i2c1 = ctx.local.i2c1;

        #[allow(unused_variables)]
        let my_delay = ctx.local.my_delay;

        let mut current_control_state = protobuf::Control::default();

        shifter.set(sr, channel2value(0), true);

        let mut start = monotonics::now();
        let mut reset = false;
        loop {
            let cycle_start = monotonics::now();
            match serial.lock(|serial| {
                protobuf_server::process(
                    serial,
                    monotonics::now().ticks(),
                    &mut current_control_state,
                    &mut [i2c1],
                    if reset {
                        reset = false;
                        true
                    } else {
                        false
                    },
                )
            }) {
                nb::Result::Ok((_, need_reset)) => {
                    let success_stop = monotonics::now();
                    defmt::debug!(
                        "Message processed ({} ms)",
                        (success_stop - cycle_start).to_millis()
                    );

                    if need_reset {
                        /* Это все не работает да еще и ломает USB на `periph.GPIOB.split();`
                        let reset_start = monotonics::now();
                        unsafe {
                            // try reset and restore i2c module
                            let periph = stm32f1xx_hal::pac::Peripherals::steal();
                            let i2c1 = periph.I2C1;
                            let rcc = periph.RCC;
                            //let mut gpiob = periph.GPIOB.split();

                            i2c1.cr1.write(|w| w.pe().clear_bit());

                            let crv_val = i2c1.ccr.read().bits();
                            let trise_val = i2c1.trise.read().bits();
                            let ccr_val = i2c1.ccr.read().bits();

                            <I2C1 as stm32f1xx_hal::rcc::Reset>::reset(&rcc);

                            i2c1.cr1.write(|w| w.pe().set_bit().swrst().set_bit());
                            i2c1.cr1.reset();

                            /*
                            let mut pins = (
                                gpiob.pb6.into_push_pull_output(&mut gpiob.crl),
                                gpiob.pb7.into_floating_input(&mut gpiob.crl),
                            );

                            pins.try_unhang_i2c(my_delay).expect("I2C unhang failed");
                            let _pins = (
                                pins.0.into_alternate_open_drain(&mut gpiob.crl),
                                pins.1.into_alternate_open_drain(&mut gpiob.crl),
                            );
                            */

                            i2c1.cr2.write(|w| w.bits(crv_val));

                            i2c1.cr1.write(|w| w.pe().clear_bit());

                            i2c1.trise.write(|w| w.bits(trise_val));
                            i2c1.ccr.write(|w| w.bits(ccr_val));

                            i2c1.cr1.modify(|_, w| w.pe().set_bit());
                        }
                        defmt::warn!(
                            "Resetting I2C1 ({} ms)",
                            (monotonics::now() - reset_start).to_millis()
                        );
                        */
                    }
                }
                nb::Result::Err(nb::Error::WouldBlock) => {
                    let step_now = monotonics::now();
                    let duration = step_now - start;

                    if duration.to_millis() > crate::config::COMMAND_TIMEOUT_MS {
                        defmt::trace!("Control timeout");
                        reset = true;
                        start = step_now;
                    }

                    // Да, именно так, иначе висяки с некоторыми USB контроллерами.
                    cortex_m::interrupt::free(|_| unsafe {
                        cortex_m::peripheral::NVIC::unmask(Interrupt::USB_HP_CAN_TX);
                        cortex_m::peripheral::NVIC::unmask(Interrupt::USB_LP_CAN_RX0);

                        //cortex_m::asm::wfi();
                    });
                }
                nb::Result::Err(nb::Error::Other(e)) => {
                    defmt::error!("Error {} while processing request", e);
                }
            }

            if current_control_state.is_updated() {
                defmt::info!("Updated control state: {}", current_control_state);

                actuator.set_state(control2pin_state(
                    current_control_state.actuator_state
                        == protobuf::messages::ActuatorState::Close,
                    config::ACTUATOR_CLOSE_STATE,
                ));

                valve.set_state(control2pin_state(
                    current_control_state.valve_state == protobuf::messages::ValveState::Atmosphere,
                    config::VALVE_ATMOSPHERE_STATE,
                ));

                shifter.set(
                    sr,
                    channel2value(current_control_state.selected_channel),
                    true,
                );

                current_control_state.reset();
            }
        }
    }
}

pub(crate) fn control2pin_state(control_state: bool, default_state: PinState) -> PinState {
    match default_state {
        PinState::High => {
            if control_state {
                PinState::High
            } else {
                PinState::Low
            }
        }
        PinState::Low => {
            if control_state {
                PinState::Low
            } else {
                PinState::High
            }
        }
    }
}

pub(crate) fn channel2value(channel: u32) -> usize {
    let v = ((channel << 1) | 1) as usize;
    match channel {
        0..=7 => v << 8,
        8..=15 => v << (4 + 8),
        _ => {
            defmt::error!("Invalid channel {}", channel);
            0
        }
    }
}

pub struct MyDelay {
    pub sys_clk: Hertz,
}

impl embedded_hal::blocking::delay::DelayUs<u32> for MyDelay {
    fn delay_us(&mut self, us: u32) {
        cortex_m::asm::delay(self.sys_clk.to_MHz() * us);
    }
}
