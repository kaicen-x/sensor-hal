use core::{
    fmt::{Debug, Formatter},
    time::Duration,
};

use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_timers::{clock::Clock, delay::Delay};

/// AHT30 working mode
#[derive(Debug)]
pub enum WorkingMode {
    /// Normal
    NOR,
    /// Cycle mode
    CYC,
    /// Command mode
    CMD,
}

/// AHT30 status
///
/// Binary bits are counted from right to left. For example, in 0b00000001, the 0th bit is 1
#[derive(Debug)]
pub struct Status {
    /// 二进制位第0位和第1位暂时空置
    _0_1: (),
    /// 校准后的电容数据是否超出CMP中断阈值范围
    ///
    /// 二进制位第2位:
    /// - 0--校准后的电容数据未超出CMP中断阈值范围
    /// - 1--校准后的电容数据超出CMP中断阈值范围
    pub cmp_interrupt: bool,
    /// 校准计算功能使能
    ///
    /// 二进制位第3位:
    /// - 0--校准计算功能被禁用，输出的数据为ADC输出的原始数据
    /// - 1--校准计算功能被启用，输出的数据为校准后的数据
    pub calibration_enabled: bool,
    /// 表示OTP存储器数据完整性测试(CRC)结果
    ///
    /// 二进制位第4位:
    /// - 0--表示完整性测试失败，表明OTP数据存在错误
    /// - 1--表示OTP存储器数据完整性测试(CRC)通过
    pub crc_ok: bool,
    /// 工作模式
    ///
    /// 二进制位第5、6位:
    /// - 00--当前处于NORmode
    /// - 01--当前处于CYCmode
    /// - 1x--当前处于CMDmode(x表示任意值)
    pub mode: WorkingMode,
    /// 是否繁忙
    ///
    /// 二进制位第7位:
    /// - 0--传感器闲，处于休眠状态
    /// - 1--传感器忙，处于正在进行测量中
    pub is_busy: bool,
}

impl Status {
    /// Parse AHT30 status
    pub fn from(data: u8) -> Self {
        Self {
            _0_1: (),
            cmp_interrupt: (data & 0b00000100) != 0,
            calibration_enabled: (data & 0b00001000) != 0,
            crc_ok: (data & 0b00010000) != 0,
            mode: if (data & 0b01000000) != 0 || (data & 0b01100000) != 0 {
                WorkingMode::CMD
            } else if (data & 0b00100000) != 0 {
                WorkingMode::CYC
            } else {
                WorkingMode::NOR
            },
            is_busy: (data & 0b10000000) != 0,
        }
    }
}

/// AHT30 sensor driver error
pub enum Error<B: I2c<SevenBitAddress>> {
    /// I2C bus raw error
    Raw(B::Error),
    /// Sensor initialization failed
    Init,
    /// The CRC8 verification of the data failed
    Crc,
    /// Sensor busy
    Busy,
}

impl<B> Debug for Error<B>
where
    B: I2c<SevenBitAddress>,
    B::Error: Debug,
{
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Raw(err) => write!(f, "I2C bus communication error, {:?}", err),
            Self::Init => write!(f, "The initialization of the AHT30 sensor failed."),
            Self::Crc => write!(f, "The CRC8 verification of the AHT30 sensor data failed."),
            Self::Busy => write!(f, "The AHT30 sensor is busy."),
        }
    }
}

#[cfg(feature = "std")]
impl<B: I2c<SevenBitAddress>> std::fmt::Display for Error<B> {
    fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
        Debug::fmt(self, f)
    }
}

#[cfg(feature = "std")]
impl<B: I2c<SevenBitAddress>> std::error::Error for Error<B> {}

/// AHT30 sensor driver
pub struct Driver<'a, C: Clock> {
    /// AHT30 7bit address
    /// - The default address is usually 0x38
    address: u8,
    /// Delay implementation for embedded_timers
    delay_impl: Delay<'a, C>,
}

impl<'a, C: Clock> Driver<'a, C> {
    /// Create an instance of the AHT30 sensor driver
    ///
    /// Note: The driver will not hold this I2C bus internally
    pub fn new<B: I2c<SevenBitAddress>>(
        clock: &'a C,
        bus: &mut B,
        address: Option<u8>,
    ) -> Result<Self, Error<B>> {
        // 处理地址
        let addr = address.unwrap_or(0x38);
        // 创建延迟实例
        let mut delay_impl = Delay::new(clock);

        // 文档明确要求上电后需要等待5ms
        delay_impl.delay(Duration::from_millis(5));

        // 发送传感器初始化命令
        bus.write(addr, &[0xBE, 0x08, 0x00])
            .map_err(|err| Error::Raw(err))?;

        // 文档明确说明传感器初始化需要10ms
        delay_impl.delay(Duration::from_millis(10));

        // 构建传感器实例
        let this = Self {
            address: addr,
            delay_impl,
        };

        // 检查传感器状态
        let status = this.read_status(bus).map_err(|err| Error::Raw(err))?;
        if !status.calibration_enabled {
            // 校准功能未启用，则传感器未初始化成功
            return Err(Error::Init);
        }

        // OK
        Ok(this)
    }

    /// Read sensor status
    ///
    /// Note: A busy sensor will not return an error. You will get the actual status of the sensor
    pub fn read_status<B: I2c<SevenBitAddress>>(&self, bus: &mut B) -> Result<Status, B::Error> {
        // 获取传感器状态
        let mut data = [0u8; 1];
        bus.read(self.address, &mut data)?;

        // 解析状态并返回
        Ok(Status::from(data[0]))
    }

    /// Calculate the CRC8 checksum
    fn calc_crc8(data: &[u8]) -> u8 {
        // 声明CRC8校验和结果
        let mut crc8_sum = 0xFF;
        // 遍历处理每一个字节
        for b in data {
            // 当前字节与已经计算的结果进行按位异或运算
            crc8_sum ^= b;
            // 再单独处理每一位二进制
            for _ in 0..8 {
                if crc8_sum & 0x80 != 0 {
                    crc8_sum = (crc8_sum << 1) ^ 0x31;
                } else {
                    crc8_sum <<= 1;
                }
            }
        }
        // OK
        crc8_sum
    }

    /// Read AHT30 sensor data
    pub fn read<B: I2c<SevenBitAddress>>(&mut self, bus: &mut B) -> Result<(f32, f32), Error<B>> {
        // 发送测量命令
        bus.write(self.address, &[0xAC, 0x33, 0x00])
            .map_err(|err| Error::Raw(err))?;
        // 根据文档要求，测量大概需要80ms才能完成
        self.delay_impl.delay(Duration::from_millis(80));

        // 读取7字节数据
        // 第1个字节（8位）: 8位二进制状态位
        // 第2~6个字节（40位）：前20位湿度 + 后20位温度
        // 第7个字节（8位）：CRC8校验字节
        let mut data = [0u8; 7];
        bus.read(self.address, &mut data)
            .map_err(|err| Error::Raw(err))?;

        // 对读取到的数据进行CRC校验
        let correct_crc8 = data[6]; // 接收到的正确的CRC8校验值
        let current_crc8 = Self::calc_crc8(&data[0..6]); // 计算出来的CRC8校验值
        if correct_crc8 != current_crc8 {
            return Err(Error::Crc);
        }

        // 解析状态信息
        let status = Status::from(data[0]);
        // 检查设备是否繁忙
        if status.is_busy {
            return Err(Error::Busy);
        }

        // 提取 20 位湿度数据
        let humidity_raw =
            ((data[1] as u32) << 12) | ((data[2] as u32) << 4) | ((data[3] as u32) >> 4);

        // 提取 20 位温度数据
        let temperature_raw =
            (((data[3] as u32) & 0b1111) << 16) | ((data[4] as u32) << 8) | data[5] as u32;

        // 转换为实际值
        let humidity = (humidity_raw as f32 / (1u32 << 20) as f32) * 100.0;
        let temperature = (temperature_raw as f32 / (1u32 << 20) as f32) * 200.0 - 50.0;

        // OK
        Ok((temperature, humidity))
    }
}
