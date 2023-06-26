// Based on https://github.com/liftoff/cupi_shift/blob/master/src/lib.rs

#[derive(Clone, Copy)]
struct ShiftRegister {
    data: usize, // e.g. 0b01010101
    pins: u8,    // Not aware of any shift registers that have more than 255 output pins
}

impl ShiftRegister {
    fn set(&mut self, data: usize) {
        self.data = data;
    }
}

pub type ShifterRef = usize;

pub struct Shifter<W, L, const N: usize>
where
    W: embedded_hal::blocking::spi::Write<u8>,
    L: embedded_hal::digital::v2::OutputPin,
{
    writer: W,
    lat: L,
    shift_registers: [ShiftRegister; N],
    count: ShifterRef,
    invert: bool,
}

impl<W, L, const N: usize> Shifter<W, L, N>
where
    W: embedded_hal::blocking::spi::Write<u8>,
    L: embedded_hal::digital::v2::OutputPin,
{
    /// Returns a new `Shifter` object that will shift out data using the given
    /// *data_pin*, *latch_pin*, and *clock_pin*.  To use a `Shifter` instance
    /// you must first call the `add()` method for each shift register you
    /// have connected in sequence.
    pub fn new(writer: W, latch_pin: L) -> Self {
        Shifter {
            writer,
            lat: latch_pin,
            shift_registers: [ShiftRegister { data: 0, pins: 0 }; N], /* FIXME */
            count: 0,
            invert: false,
        }
    }

    /// Adds a new shift register to this Shifter and returns a reference to it.
    /// You must specify the number of pins.
    pub fn add(&mut self, pins: u8) -> ShifterRef {
        use core::mem::size_of;

        assert!(pins % 8 == 0, "Number of pins must be a multiple of 8");
        assert!(
            pins < size_of::<usize>() as u8 * 8,
            "Number of pins must be less than {}",
            size_of::<usize>() * 8
        );

        let sr = ShiftRegister {
            data: 0,
            pins: pins,
        };
        self.shift_registers[self.count] = sr;
        let res = self.count;
        self.count += 1;
        res
    }

    /// Sets the *data* on the shift register at the given *sr_index*.
    /// If *apply* is `true` the change will be applied immediately.
    pub fn set(&mut self, sr_index: usize, data: usize, apply: bool) {
        for (i, sr) in self.shift_registers.iter_mut().enumerate() {
            if i == sr_index {
                sr.set(data);
                break;
            }
        }
        if apply {
            self.apply();
        }
    }

    /// Sets the given *pin* HIGH on the shift register at the given *sr_index*.
    /// If *apply* is `true` the change will be applied immediately.
    pub fn set_pin_high(&mut self, sr_index: usize, pin: u8, apply: bool) {
        for (i, sr) in self.shift_registers.iter_mut().enumerate() {
            if i == sr_index {
                let new_state = sr.data | 1 << pin;
                sr.set(new_state);
                break;
            }
        }
        if apply {
            self.apply();
        }
    }

    /// Sets the given *pin* LOW on the shift register at the given *sr_index*.
    /// If *apply* is `true` the change will be applied immediately.
    pub fn set_pin_low(&mut self, sr_index: usize, pin: u8, apply: bool) {
        for (i, sr) in self.shift_registers.iter_mut().enumerate() {
            if i == sr_index {
                let new_state = sr.data & !(1 << pin);
                sr.set(new_state);
                break;
            }
        }
        if apply {
            self.apply();
        }
    }

    /// This function will invert all logic so that HIGH is LOW and LOW is HIGH.
    /// Very convenient if you made a (very common) mistake in your wiring or
    /// you need reversed logic for other reasons.
    pub fn invert(&mut self) {
        match self.invert {
            true => self.invert = false,
            false => self.invert = true,
        }
    }

    /// Applies all current shift register states by shifting out all the stored
    /// data in each ShiftRegister object.
    pub fn apply(&mut self) {
        self.lat.set_low().ok();

        for sr in self.shift_registers.iter() {
            for n in (0..sr.pins).step_by(8) {
                let mut val = (sr.data >> n) as u8;
                if self.invert {
                    val ^= 0xFF;
                }
                self.writer.write(&[val]).ok();
            }
        }
        self.lat.set_high().ok();
    }
}
