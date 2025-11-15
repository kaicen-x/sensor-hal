use core::fmt::{Debug, Formatter};

use embedded_hal::digital::InputPin;

pub use embedded_hal::digital::PinState;

/// Transient button sensor driver
///
/// TODO: Unfortunately, I currently don't have a good way to decouple interrupts
///       from board-level drivers. Once I figure it out, I'll make up for it
pub struct TransientDriver<P: InputPin> {
    /// Button used GPIO pin
    pin: P,
    /// Input level type
    in_level: PinState,
}

impl<P: InputPin> TransientDriver<P> {
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
    pub fn state(&mut self) -> Result<bool, <P>::Error> {
        let state = self.pin.is_high()?;
        Ok(state == (self.in_level == PinState::High))
    }
}

/// Anti-shake button sensor Error
#[derive(Clone, Copy)]
pub enum AntishakeDriverError<P: InputPin> {
    /// Sensor raw error
    Raw(P::Error),
    /// Sensor not ready
    NotReady,
}

impl<P> Debug for AntishakeDriverError<P>
where
    P: InputPin,
    P::Error: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Raw(err) => write!(f, "The button signal input is incorrect, {:?}", err),
            Self::NotReady => f.write_str("The button sensor is not ready."),
        }
    }
}

#[cfg(feature = "std")]
impl<P: InputPin> std::fmt::Display for AntishakeDriverError<P> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        Debug::fmt(self, f)
    }
}

#[cfg(feature = "std")]
impl<P: InputPin> std::error::Error for AntishakeDriverError<P> {}

/// Anti-shake button sensor driver
///
/// TODO: Unfortunately, I currently don't have a good way to decouple interrupts
///       from board-level drivers. Once I figure it out, I'll make up for it
pub struct AntishakeDriver<P: InputPin> {
    /// Button used GPIO pin
    pin: P,
    /// Input level type
    in_level: PinState,
    /// Last state
    /// - True: Press the button
    /// - False: Release the button
    last_state: bool,
}

impl<P: InputPin> AntishakeDriver<P> {
    /// Create an instance of the switch sensor driver
    ///
    /// - in_level: The level input when the switch is pressed
    pub fn new(mut pin: P, in_level: PinState) -> Result<Self, AntishakeDriverError<P>> {
        // 尝试最多10次获取初始的按钮状态
        let mut init_state = None;
        for _ in 0..10 {
            if let Some(state) =
                Self::read(&mut pin, in_level).map_err(|err| AntishakeDriverError::Raw(err))?
            {
                init_state = Some(state);
                break;
            }
        }
        if let Some(state) = init_state {
            // OK
            Ok(Self {
                pin,
                in_level,
                last_state: state,
            })
        } else {
            Err(AntishakeDriverError::NotReady)
        }
    }

    /// Read the status of the button
    ///
    /// - Some(True): Press the button
    /// - Some(False): Release the button
    fn read(pin: &mut P, in_level: PinState) -> Result<Option<bool>, P::Error> {
        let mut cnt: u8 = 0;
        for _ in 0..8 {
            if pin.is_high()? {
                cnt = (cnt << 1) | 1;
            } else {
                cnt = cnt << 1;
            }
        }
        if cnt == 0xFF {
            // 稳定的高电平
            Ok(Some(in_level == PinState::High))
        } else if cnt == 0x00 {
            // 稳定的低电平
            Ok(Some(in_level == PinState::Low))
        } else {
            // 不稳定，需要重新采集
            Ok(None)
        }
    }

    /// Get the status of the button
    ///
    /// - True: Press the button
    /// - False: Release the button
    pub fn state(&mut self) -> Result<bool, P::Error> {
        if let Some(state) = Self::read(&mut self.pin, self.in_level)? {
            // 稳定的,更新上一次状态
            self.last_state = state;
        }
        // 返回旧的状态
        Ok(self.last_state)
    }
}
