use embedded_hal::digital::OutputPin;

pub use embedded_hal::digital::PinState;

/// What is the goal of encapsulating the switch driver?
/// Generally, controlling an switch is very simple and does not require a driver library at all.
/// However, when the switch's lighting state suddenly changes from high level to low level,
/// do you have to modify each calling place one by one?    This switch driver is designed to solve this problem.
pub struct Driver<P: OutputPin> {
    /// Switch used GPIO pin
    pin: P,
    /// Output level type
    out_level: PinState,
}

impl<P: OutputPin> Driver<P> {
    /// Create an instance of the switch sensor driver
    ///
    /// - out_level: What level should be used to make the switch on
    pub fn new(pin: P, out_level: PinState) -> Self {
        Self { pin, out_level }
    }

    /// On the switch sensor
    pub fn on(&mut self) -> Result<(), P::Error> {
        self.pin.set_state(self.out_level)
    }

    /// Off the switch sensor
    pub fn off(&mut self) -> Result<(), P::Error> {
        match self.out_level {
            PinState::High => self.pin.set_low(),
            PinState::Low => self.pin.set_high(),
        }
    }
}
