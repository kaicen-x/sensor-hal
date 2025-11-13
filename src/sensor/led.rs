use embedded_hal::digital::{OutputPin, PinState};

/// LED Sensor Driver
///
/// What is the goal of encapsulating the LED driver?
/// Generally, controlling an LED is very simple and does not require a driver library at all.
/// However, when the LED's lighting state suddenly changes from high level to low level,
/// do you have to modify each calling place one by one? This LED driver is designed to solve this problem.
pub struct Driver<Pin> {
    /// LED used GPIO pin
    pin: Pin,
    /// Output level type
    out_level: PinState,
}

impl<Pin: OutputPin> Driver<Pin> {
    /// Create an instance of the LED sensor driver
    pub fn new(pin: Pin, out_level: PinState) -> Self {
        Self { pin, out_level }
    }

    /// Light up the LED sensor
    pub fn on(&mut self) -> Result<(), Pin::Error> {
        self.pin.set_state(self.out_level)
    }

    /// Extinguish the LED sensor
    pub fn off(&mut self) -> Result<(), Pin::Error> {
        match self.out_level {
            PinState::High => self.pin.set_low(),
            PinState::Low => self.pin.set_high(),
        }
    }
}
