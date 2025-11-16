use core::fmt::{Debug, Formatter};
use core::time::Duration;

use embedded_hal::digital::{InputPin, OutputPin};
use embedded_timers::clock::Clock;
use embedded_timers::delay::Delay;

/// HX711 channel and gain
#[derive(Clone, Copy)]
pub enum ChannelGain {
    /// Channel: A, Gain: 128
    /// - Send one pulse
    ChannelA128 = 1,
    /// Channel: B, Gain: 32
    /// - Send two pulses
    ChannelB32 = 2,
    /// Channel: A, Gain: 64
    /// - Send three pulses
    ChannelA64 = 3,
}

/// HX711 sensor Error
pub enum Error<IP: InputPin, OP: OutputPin> {
    /// Digital I/O input error
    Input(IP::Error),
    /// Digital I/O output error
    Output(OP::Error),
    /// Sensor not ready
    NotReady,
}

impl<IP, OP> Debug for Error<IP, OP>
where
    IP: InputPin,
    IP::Error: Debug,
    OP: OutputPin,
    OP::Error: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Input(err) => write!(f, "The HX711 data signal input is incorrect, {:?}", err),
            Self::Output(err) => write!(f, "The HX711 data signal ouput is incorrect, {:?}", err),
            Self::NotReady => write!(f, "The HX711 sensor is not ready."),
        }
    }
}

#[cfg(feature = "std")]
impl<IP: InputPin, OP: OutputPin> std::fmt::Display for Error<IP, OP> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        Debug::fmt(self, f)
    }
}

#[cfg(feature = "std")]
impl<IP: InputPin, OP: OutputPin> std::error::Error for Error<IP, OP> {}

/// HX711 Sensor Driver
pub struct Driver<'a, C: Clock, I: InputPin, O: OutputPin> {
    /// Clock used GPIO pin
    clock_pin: O,
    /// Data used GPIO pin
    data_pin: I,
    /// Channel and Gain config
    channel_gain: ChannelGain,
    /// Delay implementation for embedded_timers
    delay_impl: Delay<'a, C>,
}

impl<'a, C: Clock, I: InputPin, O: OutputPin> Driver<'a, C, I, O> {
    /// Create an instance of the HX711 sensor driver
    pub fn new(
        clock: &'a C,
        mut clock_pin: O,
        data_pin: I,
        channel_gain: ChannelGain,
    ) -> Result<Self, O::Error> {
        // 拉低时钟信号电平，使芯片上电
        clock_pin.set_low()?;
        // OK
        Ok(Self {
            clock_pin,
            data_pin,
            channel_gain,
            delay_impl: Delay::new(clock),
        })
    }

    /// Check if the HX711 sensor is ready
    pub fn is_ready(&mut self) -> Result<bool, I::Error> {
        // 当DATA引脚为高电平时，表示数据未就绪
        // 一旦为低电平，表示数据就绪，可以读取数据
        self.data_pin.is_low()
    }

    /// Read HX711 sensor output data
    pub fn read(&mut self) -> Result<i32, Error<I, O>> {
        // 检查数模转换芯片是否就绪
        let is_ready = self.is_ready().map_err(|err| Error::Input(err))?;
        if !is_ready {
            return Err(Error::NotReady);
        }

        // 读取到的原始数据
        let mut raw_data: u32 = 0;

        // 读取24位数据
        for _ in 0..24 {
            // 发送时钟信号高电平，表示要开始读取一位数据
            self.clock_pin
                .set_high()
                .map_err(|err| Error::Output(err))?;
            // 维持高电平信号1微秒能保证时钟信号到达
            self.delay_impl.delay(Duration::from_micros(1));

            // 读取数据引脚的电平
            if self.data_pin.is_high().map_err(|err| Error::Input(err))? {
                // 高电平表示读取到的二进制位为1
                // 把原来的数据左移一位，然后将末尾一位置为1
                raw_data = (raw_data << 1) | 1
            } else {
                // 低电平表示读取到的二进制位为0
                // 把原来的数据左移一位，末尾一位自动就变为0了
                raw_data = raw_data << 1
            }

            // 发送时钟信号低电平，表示读取完一位数据
            self.clock_pin.set_low().map_err(|err| Error::Output(err))?;
            // 维持低电平信号1微秒能保证时钟信号到达
            self.delay_impl.delay(Duration::from_micros(1));
        }

        // 设置通道和增益
        // 告知HX711下一次应该发送哪一个通道（A、B两个通道）的数据，并且增益（A支持128和64，B只支持32）是多少.
        // A通道增益128: 发送一个脉冲
        // B通道增益32: 发送二个脉冲
        // A通道增益64: 发送三个脉冲
        for _ in 0..(self.channel_gain as u8) {
            // 发送时钟信号高电平
            self.clock_pin
                .set_high()
                .map_err(|err| Error::Output(err))?;
            // 维持高电平信号1微秒能保证时钟信号到达
            self.delay_impl.delay(Duration::from_micros(1));
            // 发送时钟信号低电平
            self.clock_pin.set_low().map_err(|err| Error::Output(err))?;
            // 维持高电平信号1微秒能保证时钟信号到达
            self.delay_impl.delay(Duration::from_micros(1));
        }

        // 确保我们只处理低24位，屏蔽掉可能的高8位
        // 0x00FFFFFF 是 0000 0000 1111 1111 1111 1111 1111 1111
        raw_data &= 0x00FFFFFF;
        // 检查符号位（最高位）
        if (raw_data & 0x00800000) != 0 {
            // 0x00800000 是 0000 0000 1000 0000 0000 0000 0000 0000
            // 如果符号位是1（负数），则进行符号扩展（将高8位置1）
            // 0xFF000000 是 1111 1111 0000 0000 0000 0000 0000 0000
            Ok((raw_data | 0xFF000000) as i32)
        } else {
            // 如果符号位是0（正数），高8位已经是0，无需操作。
            // 或者可以显式地确保高8位为0
            // 0x00FFFFFF 是 0000 0000 1111 1111 1111 1111 1111 1111
            // rawData &= 0x00FFFFFF;
            // OK
            Ok(raw_data as i32)
        }
    }

    /// Disable HX711 sensor
    pub fn disable(&mut self) -> Result<(), O::Error> {
        // 时钟引脚保持60微秒以上即可使HX711芯片断电
        self.clock_pin.set_high()?;
        self.delay_impl.delay(Duration::from_micros(60));
        Ok(())
    }

    /// Enable HX711 sensor
    pub fn enable(&mut self) -> Result<(), O::Error> {
        // 将时钟信号设为低电平，HX711芯片上电，
        self.clock_pin.set_low()
    }

    /// Reset HX711 sensor
    pub fn reset(&mut self) -> Result<(), O::Error> {
        // 断电再上电即可实现重置
        self.disable()?;
        self.enable()
    }

    /// Setting HX711 sensor channel and gain
    pub fn set_channel_gain(&mut self, gain: ChannelGain) {
        // 设置通道和增益后，根据厂家的文档描述，需要采集4次以上新的数据才会稳定
        self.channel_gain = gain;
    }
}
