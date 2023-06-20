#![no_main]
#![no_std]
#![feature(array_zip)]
#![feature(macro_metavar_expr)]

mod config;
mod hw;
mod support;

use defmt_rtt as _; // global logger
use panic_probe as _;

use rtic::app;

use stm32f1xx_hal::afio::AfioExt;
use stm32f1xx_hal::flash::FlashExt;
use stm32f1xx_hal::gpio::GpioExt;
use stm32f1xx_hal::rcc::{HPre, PPre};
use stm32f1xx_hal::time::Hertz;
use stm32f1xx_hal::usb::{Peripheral, UsbBus, UsbBusType};
use usb_device::prelude::{UsbDevice, UsbDeviceBuilder};

use stm32f1xx_hal::pac::Interrupt;

use usbd_serial::SerialPort;

use systick_monotonic::Systick;

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

#[app(device = stm32f1xx_hal::pac, peripherals = true, dispatchers = [RTCALARM])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        usb_device: UsbDevice<'static, UsbBusType>,
        serial: SerialPort<'static, UsbBus<Peripheral>>,
    }

    #[local]
    struct Local {}

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<{ config::SYSTICK_RATE_HZ }>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        static mut USB_BUS: Option<usb_device::bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = ctx.device.FLASH.constrain();

        let mut gpioa = ctx.device.GPIOA.split();
        let mut gpiob = ctx.device.GPIOB.split();
        //let mut gpioc = ctx.device.GPIOC.split();

        let mut afio = ctx.device.AFIO.constrain();
        let (pa15, _pb3, _pb4) = afio.mapr.disable_jtag(gpioa.pa15, gpiob.pb3, gpiob.pb4);

        let mut usb_pull_up = pa15.into_push_pull_output_with_state(
            &mut gpioa.crh,
            if !config::USB_PULLUP_ACTVE_LEVEL {
                stm32f1xx_hal::gpio::PinState::High
            } else {
                stm32f1xx_hal::gpio::PinState::Low
            },
        );

        let clocks = HighPerformanceClockConfigProvider::freeze(&mut flash.acr);

        let mono = Systick::new(ctx.core.SYST, clocks.sysclk().to_Hz());

        let usb = Peripheral {
            usb: ctx.device.USB,
            pin_dm: gpioa.pa11,
            pin_dp: gpioa.pa12,
        };

        unsafe {
            USB_BUS.replace(UsbBus::new(usb));
        }

        let serial = SerialPort::new(unsafe { USB_BUS.as_ref().unwrap_unchecked() });

        let usb_dev = UsbDeviceBuilder::new(
            unsafe { USB_BUS.as_ref().unwrap_unchecked() },
            usb_device::prelude::UsbVidPid(0x16c0, 0x27dd),
        )
        .manufacturer("SCTBElpa")
        .product("laser-setup")
        .serial_number("1")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

        //---------------------------------------------------------------------

        usb_pull_up.toggle(); // enable USB

        //---------------------------------------------------------------------

        (
            Shared {
                usb_device: usb_dev,
                serial,
            },
            Local {},
            init::Monotonics(mono),
        )
    }

    //-------------------------------------------------------------------------

    #[task(binds = USB_HP_CAN_TX, shared = [usb_device, serial], priority = 1)]
    fn usb_tx(ctx: usb_tx::Context) {
        let mut usb_device = ctx.shared.usb_device;
        let mut serial = ctx.shared.serial;

        if !(&mut usb_device, &mut serial).lock(move |usb_device, serial| {
            super::usb_poll(usb_device, serial /*, gcode_pusher, request_pusher*/)
        }) {
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_HP_CAN_TX);
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_LP_CAN_RX0);
        }
    }

    #[task(binds = USB_LP_CAN_RX0, shared = [usb_device, serial], priority = 1)]
    fn usb_rx0(ctx: usb_rx0::Context) {
        let mut usb_device = ctx.shared.usb_device;
        let mut serial = ctx.shared.serial;

        if !(&mut usb_device, &mut serial).lock(move |usb_device, serial| {
            super::usb_poll(usb_device, serial /*, gcode_pusher, request_pusher*/)
        }) {
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_HP_CAN_TX);
            cortex_m::peripheral::NVIC::mask(Interrupt::USB_LP_CAN_RX0);
        }
    }

    //-------------------------------------------------------------------------

    //#[idle(shared=[serial], local = [])]
    //fn idle(ctx: idle::Context) -> ! {
    //}
}

fn usb_poll<B: usb_device::bus::UsbBus /*, GP, RP*/>(
    usb_dev: &mut usb_device::prelude::UsbDevice<'static, B>,
    serial: &mut usbd_serial::SerialPort<'static, B>,
    /*
    gcode_pusher: GP,
    request_pusher: RP,
    */
) -> bool
where
    /*
    GP: FnMut(gcode::GCode) -> Result<(), gcode::GCode>,
    RP: FnMut(gcode::Request) -> Result<(), gcode::Request>,
    */
    B: usb_device::bus::UsbBus,
{
    //use heapless::String;

    //static mut BUF: String<{ gcode::MAX_LEN }> = String::new();

    if !usb_dev.poll(&mut [serial]) {
        return true;
    }
    /*
    let trimm_buff = |trimm_size| unsafe {
        if trimm_size > 0 {
            if trimm_size == BUF.len() {
                BUF.clear()
            } else {
                let new_buf = BUF.chars().skip(trimm_size).collect();
                BUF = new_buf;
            }
        }
    };

    match gcode::serial_process(serial, unsafe { &mut BUF }, gcode_pusher, request_pusher) {
        Ok(trimm_size) => trimm_buff(trimm_size),
        Err(SerialErrResult::OutOfMemory) => {
            unsafe { BUF.clear() };
            serial.write(b"Command too long! (150)").unwrap();
        }
        Err(SerialErrResult::Incomplead(_trimm_size)) => {
            //serial.write(b"Command buffer full").unwrap();
            return false;
        }
        _ => {}
    }
    */

    true
}
