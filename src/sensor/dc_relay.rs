use embedded_hal::{digital::OutputPin, pwm::SetDutyCycle};

pub use embedded_hal::digital::PinState;

/// DC relay sensor driver
///
/// What is the goal of encapsulating the dc relay driver?
/// Generally, controlling an dc relay is very simple and does not require a driver library at all.
/// However, when the dc relay's lighting state suddenly changes from high level to low level,
/// do you have to modify each calling place one by one? This dc relay driver is designed to solve this problem.
pub struct Driver<P: OutputPin> {
    /// DC relay used GPIO pin
    pin: P,
    /// Output level type
    out_level: PinState,
}

impl<P: OutputPin> Driver<P> {
    /// Create an instance of the dc relay sensor driver
    ///
    /// - out_level: What level should be used to make the dc relay on
    pub fn new(pin: P, out_level: PinState) -> Self {
        Self { pin, out_level }
    }

    /// On the dc relay sensor
    pub fn on(&mut self) -> Result<(), P::Error> {
        self.pin.set_state(self.out_level)
    }

    /// Off the dc relay sensor
    pub fn off(&mut self) -> Result<(), P::Error> {
        match self.out_level {
            PinState::High => self.pin.set_low(),
            PinState::Low => self.pin.set_high(),
        }
    }
}

/// DC relay sensor pwm driver
/// 
/// The PWM driver is optional, and you can directly use your board-level PWM driver. 
/// This PWM driver is merely designed to make your code look more uniform. 
/// Of course, it internally uses #[inline] to call the board-level PWM driver, 
/// and there will be no performance loss at all
pub struct PwmDriver<P: SetDutyCycle> {
    /// DC relay used GPIO pin
    pin: P,
}

impl<P: SetDutyCycle> PwmDriver<P> {
    /// Create an instance of the dc relay sensor driver
    pub fn new(pin: P) -> Self {
        Self { pin }
    }

    /// Get the maximum duty cycle value.
    ///
    /// This value corresponds to a 100% duty cycle.
    #[inline]
    pub fn max_duty_cycle(&self) -> u16 {
        self.pin.max_duty_cycle()
    }

    /// Set the duty cycle to `duty / max_duty`.
    ///
    /// The caller is responsible for ensuring that the duty cycle value is less than or equal to the maximum duty cycle value,
    /// as reported by [`max_duty_cycle`].
    ///
    /// [`max_duty_cycle`]: SetDutyCycle::max_duty_cycle
    #[inline]
    pub fn set_duty_cycle(&mut self, duty: u16) -> Result<(), P::Error> {
        self.pin.set_duty_cycle(duty)
    }

    /// Set the duty cycle to 0%, or always inactive.
    #[inline]
    pub fn set_duty_cycle_fully_off(&mut self) -> Result<(), P::Error> {
        self.pin.set_duty_cycle_fully_off()
    }

    /// Set the duty cycle to 100%, or always active.
    #[inline]
    pub fn set_duty_cycle_fully_on(&mut self) -> Result<(), P::Error> {
        self.pin.set_duty_cycle_fully_on()
    }

    /// Set the duty cycle to `num / denom`.
    ///
    /// The caller is responsible for ensuring that `num` is less than or equal to `denom`,
    /// and that `denom` is not zero.
    #[inline]
    pub fn set_duty_cycle_fraction(&mut self, num: u16, denom: u16) -> Result<(), P::Error> {
        self.pin.set_duty_cycle_fraction(num, denom)
    }

    /// Set the duty cycle to `percent / 100`
    ///
    /// The caller is responsible for ensuring that `percent` is less than or equal to 100.
    #[inline]
    pub fn set_duty_cycle_percent(&mut self, percent: u8) -> Result<(), P::Error> {
        self.pin.set_duty_cycle_percent(percent)
    }
}
