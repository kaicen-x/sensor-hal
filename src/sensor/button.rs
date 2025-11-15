use embedded_hal::digital::InputPin;

pub use embedded_hal::digital::PinState;

/// Button sensor driver
/// 
/// TODO: Unfortunately, I currently don't have a good way to decouple interrupts 
///       from board-level drivers. Once I figure it out, I'll make up for it
pub struct Driver<P: InputPin> {
    /// Button used GPIO pin
    pin: P,
    /// Input level type
    in_level: PinState,
}

impl<P: InputPin> Driver<P> {
    /// Create an instance of the switch sensor driver
    ///
    /// - in_level: The level input when the switch is pressed
    pub fn new(pin: P, in_level: PinState) -> Self {
        Self { pin, in_level }
    }

    /// Get the status of the button
    /// 
    /// - True: Press the button
    /// - False: Release the button
    pub fn state(&mut self) -> Result<bool, P::Error> {
        let level = self.pin.is_low()?;
        Ok(level == (self.in_level == PinState::Low))
    }
}
