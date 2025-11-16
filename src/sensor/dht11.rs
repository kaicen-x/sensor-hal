use core::{
    fmt::{Debug, Formatter},
    hint::spin_loop,
    time::Duration,
};

use embedded_hal::digital::{InputPin, OutputPin, PinState};
use embedded_timers::{clock::Clock, delay::Delay};

/// DHT11 sensor Error
#[derive(Clone, Copy)]
pub enum Error<P: InputPin + OutputPin> {
    /// Digital I/O input error
    Input(P::Error),
    /// Digital I/O output error
    Output(P::Error),
    /// Sensor not ready
    NotReady,
    /// Check sum error
    CheckSum,
}

impl<P> Debug for Error<P>
where
    P: InputPin + OutputPin,
    P::Error: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Input(err) => write!(f, "The DHT11 data signal input is incorrect, {:?}.", err),
            Self::Output(err) => write!(f, "The DHT11 data signal ouput is incorrect, {:?}.", err),
            Self::NotReady => write!(f, "The DHT11 sensor is not ready."),
            Self::CheckSum => {
                write!(f, "The checksum of the input data of the DHT11 sensor is incorrect.")
            }
        }
    }
}

#[cfg(feature = "std")]
impl<P: InputPin + OutputPin> std::fmt::Display for Error<P> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        Debug::fmt(self, f)
    }
}

#[cfg(feature = "std")]
impl<P: InputPin + OutputPin> std::error::Error for Error<P> {}

/// DHT11 Sensor Driver
pub struct Driver<'a, P: InputPin + OutputPin, C: Clock> {
    /// 1-Wire used GPIO pin
    pin: P,
    /// External clock implementation
    clock_impl: &'a C,
    /// Delay implementation for embedded_timers
    delay_impl: Delay<'a, C>,
}

impl<'a, P: InputPin + OutputPin, C: Clock> Driver<'a, P, C> {
    /// Create an instance of the DHT11 sensor driver
    pub fn new(mut pin: P, clock_impl: &'a C) -> Result<Self, P::Error> {
        // DHT11上电后（DHT11上电后要等待 1S 以越过不稳定状态在此期间不能发送任何指令），测试环境
        // 温湿度数据，并记录数据，同时 DHT11的DATA数据线由上拉电阻拉高一直保持高电平；此时 DHT11的
        // DATA 引脚处于输入状态，时刻检测外部信号。

        // 拉高电平(使总线处于空闲状态)
        pin.set_high()?;
        // 创建延迟等待实例
        let mut delay = embedded_timers::delay::Delay::new(clock_impl);
        // 拉高至少1秒
        delay.delay(Duration::from_secs(1));
        // OK
        Ok(Self {
            pin,
            clock_impl,
            delay_impl: delay,
        })
    }

    /// Wait for the level signal within the specified time range
    fn wait_sensor_signal(
        &mut self,
        target_state: PinState,
        timeout: Duration,
    ) -> Result<Duration, Error<P>> {
        // 获取开始时间点
        let start = self.clock_impl.now();
        // 目标电平是否为低电平
        let target_state_is_low = target_state == PinState::Low;
        // 循环检查
        while (self.clock_impl.now() - start) < timeout {
            // 获取当前电平状态
            let is_low = self.pin.is_low().map_err(|err| Error::Input(err))?;
            // 状态是否一致了
            if is_low == target_state_is_low {
                // 已经接收到信号了，立即返回
                return Ok(self.clock_impl.now() - start);
            }
            // 降低CPU功耗
            spin_loop();
        }
        // 默认为超时了
        Err(Error::NotReady)
    }

    /// Read sensor data
    ///
    /// Note:
    /// - According to the document description, the read data is the result of the previous measurement.
    /// - If real-time measurement is required, please call it twice or more
    pub fn read(&mut self) -> Result<(f32, f32), Error<P>> {
        // 把数据总线（SDA）拉低一段时间至少18ms（最大不得超过30ms），通知传感器准备数据
        self.pin.set_low().map_err(|err| Error::Output(err))?;
        self.delay_impl.delay(Duration::from_millis(20));

        // 将数据总线切换为输入模式，由于上拉电阻的存在，数据总线会自动变为高电平
        // 等待传感器把数据总线（SDA）拉低83µs，再拉高87µs以响应主机的起始信号
        // 0. 等待低电平开始, 超时长一点即可
        self.wait_sensor_signal(PinState::Low, Duration::from_micros(1000))?;
        // 1. 等待83us的低电平结束（即等待高电平开始）, 超时稍微比83us高一点即可
        self.wait_sensor_signal(PinState::High, Duration::from_micros(90))?;
        // 2. 等待87us的高电平结束（即等待低电平开始）, 超时稍微比87us高一点即可
        self.wait_sensor_signal(PinState::Low, Duration::from_micros(95))?;

        // 立即读取40位数据
        // 收到主机信号后，从机一次性从SDA串出40bit，高位先出
        // 数据格式:
        // (8bit 湿度整数数据)  + (8bit 湿度小数数据)  + (8bit 温度整数数据)  + (8)bit 温度小数数据)  + (8bit 校验位)。
        // 注：其中湿度小数部分为0。
        let mut data = [0u8; 5];
        // 将数据分为5组来读
        for byte in 0..5 {
            // 每组数据都8位
            for bit in 0..8 {
                // 位数据“0”的格式为: 54us的低电平和23-27us的高电平
                // 位数据“1”的格式为: 54us的低电平加68-74us的高电平

                // 1. 等待54us的低电平信号结束（即等待高电平开始）, 超时稍微比54us高一点即可
                self.wait_sensor_signal(PinState::High, Duration::from_micros(60))?;
                // 2. 等待74us（最长）的高电平信号（即等待低电平开始）, 超时稍微比74us高一点即可
                let high_time =
                    self.wait_sensor_signal(PinState::Low, Duration::from_micros(80))?;
                // 3. 根据高电平的时长来判断位数据是"0"还是"1", 这里30是给的冗余判断，稍微大于位数据"0"的范围即可
                if high_time > Duration::from_micros(30) {
                    data[byte] |= 1 << (7 - bit);
                }
            }
        }

        // 校验数据
        let checksum = data[0]
            .wrapping_add(data[1])
            .wrapping_add(data[2])
            .wrapping_add(data[3]);
        if checksum != data[4] {
            return Err(Error::CheckSum);
        }

        // 转换相对湿度(%RH)，DHT11返回的相对湿度小数部分始终为0，故忽略
        let humidity = data[0] as f32;
        // 转换摄氏温度(℃)
        let mut temperature = data[2] as f32 + ((data[3] & 0b01111111) as f32 * 0.1);
        if data[3] & 0x80 != 0 {
            temperature = -temperature;
        }

        // OK
        Ok((temperature, humidity))
    }
}
