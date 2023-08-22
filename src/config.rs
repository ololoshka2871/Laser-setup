use stm32f1xx_hal::gpio::PinState;

//-----------------------------------------------------------------------------

pub const XTAL_FREQ: u32 = 16_000_000;

//-----------------------------------------------------------------------------

// usb pull up
pub const USB_PULLUP_ACTVE_LEVEL: PinState = PinState::High;

//-----------------------------------------------------------------------------

// systick rate
pub const SYSTICK_RATE_HZ: u32 = 1_000;

// command timeout
pub const COMMAND_TIMEOUT_MS: u64 = 100;

//-----------------------------------------------------------------------------

// heap size
pub const HEAP_SIZE: usize = 1024;

//-----------------------------------------------------------------------------

// Channels count
pub const CHANNELS_COUNT: u32 = 16;

//-----------------------------------------------------------------------------
// Catuator close state
pub const ACTUATOR_CLOSE_STATE: PinState = PinState::Low;

// Clapan close state
pub const VALVE_ATMOSPHERE_STATE: PinState = PinState::Low;