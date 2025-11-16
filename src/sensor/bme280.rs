use core::{
    fmt::{Debug, Formatter},
    time::Duration,
};

use embedded_hal::i2c::{I2c, SevenBitAddress};
use embedded_timers::{clock::Clock, delay::Delay};

/// BME280传感器校准参数结构体
///
/// 该结构体存储了从传感器 NVM 中读取的所有校准参数，用于
/// 温度、压力和湿度测量的精确补偿计算。这些参数在生产过程
/// 中经过精密校准，确保传感器的高精度性能。
///
/// # 存储分布
/// - 温度/压力参数: 地址 0x88-0xA1 (24字节)
/// - 湿度参数: 地址 0xA1, 0xE1-0xE7 (7字节)
///
/// # 重要性
/// 校准参数消除了传感器制造差异，提供：
/// - 温度依赖性补偿
/// - 非线性响应校正  
/// - 长期稳定性保证
/// - 交叉敏感性消除
struct Calibration {
    /// 温度校准参数组
    pub dig_t1: u16,
    pub dig_t2: i16,
    pub dig_t3: i16,

    /// 压力校准参数组
    pub dig_p1: u16,
    pub dig_p2: i16,
    pub dig_p3: i16,
    pub dig_p4: i16,
    pub dig_p5: i16,
    pub dig_p6: i16,
    pub dig_p7: i16,
    pub dig_p8: i16,
    pub dig_p9: i16,

    /// 湿度校准参数组
    pub dig_h1: u8,
    pub dig_h2: i16,
    pub dig_h3: u8,
    pub dig_h4: i16,
    pub dig_h5: i16,
    pub dig_h6: i8,
}

impl Calibration {
    /// Parse BME280 Calibration params
    pub fn from(tp_calib: &[u8; 24], h_calib: &[u8; 7]) -> Self {
        Self {
            // 温度、气压校准参数
            dig_t1: u16::from_le_bytes([tp_calib[0], tp_calib[1]]),
            dig_t2: i16::from_le_bytes([tp_calib[2], tp_calib[3]]),
            dig_t3: i16::from_le_bytes([tp_calib[4], tp_calib[5]]),
            dig_p1: u16::from_le_bytes([tp_calib[6], tp_calib[7]]),
            dig_p2: i16::from_le_bytes([tp_calib[8], tp_calib[9]]),
            dig_p3: i16::from_le_bytes([tp_calib[10], tp_calib[11]]),
            dig_p4: i16::from_le_bytes([tp_calib[12], tp_calib[13]]),
            dig_p5: i16::from_le_bytes([tp_calib[14], tp_calib[15]]),
            dig_p6: i16::from_le_bytes([tp_calib[16], tp_calib[17]]),
            dig_p7: i16::from_le_bytes([tp_calib[18], tp_calib[19]]),
            dig_p8: i16::from_le_bytes([tp_calib[20], tp_calib[21]]),
            dig_p9: i16::from_le_bytes([tp_calib[22], tp_calib[23]]),

            // 湿度校准参数
            dig_h1: h_calib[0],
            dig_h2: i16::from_le_bytes([h_calib[1], h_calib[2]]),
            dig_h3: h_calib[3],
            dig_h4: (i16::from(h_calib[4]) << 4) | (i16::from(h_calib[5]) & 0x0F),
            dig_h5: (i16::from(h_calib[6]) << 4) | (i16::from(h_calib[5]) >> 4),
            dig_h6: h_calib[6] as i8,
        }
    }
}

/// BME280 sensor driver error
pub enum Error<B: I2c<SevenBitAddress>> {
    /// I2C bus raw error
    Raw(B::Error),
    /// Sensor initialization failed
    Init,
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
            Self::Init => write!(f, "The initialization of the BME280 sensor failed."),
            Self::Busy => write!(f, "The BME280 sensor is busy."),
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

/// BME280 sensor driver
pub struct Driver<'a, C: Clock> {
    /// BME280 7bit address
    /// - The default address is usually 0x76
    address: u8,
    /// BME280 Calibration params
    calib: Calibration,
    /// Delay implementation for embedded_timers
    delay_impl: Delay<'a, C>,
}

impl<'a, C: Clock> Driver<'a, C> {
    /// Read calibration data
    fn read_calibration_data<B: I2c<SevenBitAddress>>(
        bus: &mut B,
        address: u8,
    ) -> Result<Calibration, B::Error> {
        // 读取温度/压力校准参数 (0x88-0x9F)
        let mut tp_calib = [0u8; 24];
        bus.write_read(address, &[0x88], &mut tp_calib)?;
        // 读取湿度校准参数 (0xA1, 0xE1-0xE7)
        let mut h_calib = [0u8; 7];
        bus.write_read(address, &[0xA1], &mut h_calib[0..1])?;
        bus.write_read(address, &[0xE1], &mut h_calib[1..7])?;
        // OK
        Ok(Calibration::from(&tp_calib, &h_calib))
    }

    /// Create an instance of the BME280 sensor driver
    ///
    /// Note: The driver will not hold this I2C bus internally
    pub fn new<B: I2c<SevenBitAddress>>(
        clock: &'a C,
        bus: &mut B,
        address: Option<u8>,
    ) -> Result<Self, Error<B>> {
        // 处理地址
        let addr = address.unwrap_or(0x76);
        // 创建延迟实例
        let mut delay_impl = Delay::new(clock);

        // 文档明确要求上电后需要等待2ms以上
        delay_impl.delay(Duration::from_millis(3));

        // 检查传感器是否就绪
        let mut status = [0u8];
        bus.write_read(addr, &[0xF3], &mut status)
            .map_err(|err| Error::Raw(err))?;
        // 检查状态
        if status[0] & 0x01 != 0 {
            return Err(Error::Init);
        }

        // 读取校准参数
        let calib = Self::read_calibration_data(bus, addr).map_err(|err| Error::Raw(err))?;

        // 配置湿度采样率 (osrs_h = 1x)
        bus.write(addr, &[0xF2, 0x01])
            .map_err(|err| Error::Raw(err))?;
        delay_impl.delay(Duration::from_millis(10));
        // 配置温度、压力采样率 (osrs_t = 1x, osrs_p = 1x) 和正常模式
        bus.write(addr, &[0xF4, 0x27])
            .map_err(|err| Error::Raw(err))?; // 00100111 = 0x27
        delay_impl.delay(Duration::from_millis(10));
        // 配置滤波器关闭，待机时间 0.5ms
        bus.write(addr, &[0xF5, 0x00])
            .map_err(|err| Error::Raw(err))?;
        delay_impl.delay(Duration::from_millis(10));

        // OK
        Ok(Self {
            address: addr,
            calib,
            delay_impl,
        })
    }

    /// Read ADC raw data
    fn read_raw_data<B: I2c<SevenBitAddress>>(
        &self,
        bus: &mut B,
    ) -> Result<(i32, i32, i32), B::Error> {
        // 声明缓冲区
        let mut data = [0u8; 8];

        // 读取原始数据
        bus.write_read(self.address, &[0xF7], &mut data)?;

        // 解析20位压力数据 (0xF7-0xF9)
        let press_msb = data[0] as i32;
        let press_lsb = data[1] as i32;
        let press_xlsb = data[2] as i32;
        let press_raw = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4);

        // 解析20位温度数据 (0xFA-0xFC)
        let temp_msb = data[3] as i32;
        let temp_lsb = data[4] as i32;
        let temp_xlsb = data[5] as i32;
        let temp_raw = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);

        // 解析16位湿度数据 (0xFD-0xFE)
        let hum_msb = data[6] as i32;
        let hum_lsb = data[7] as i32;
        let hum_raw = (hum_msb << 8) | hum_lsb;

        // OK
        Ok((press_raw, temp_raw, hum_raw))
    }

    /// BME280温度补偿函数
    ///
    /// **功能描述**
    /// 根据数据手册 4.2.3 节的温度补偿公式，将原始 ADC 温度值转换为
    /// 摄氏度温度，并生成用于压力/湿度补偿的 t_fine 值。
    ///
    /// **参数**
    /// - `adc_t`: 从寄存器 0xFA-0xFC 读取的原始20位温度ADC值
    ///
    /// **返回**
    /// - `(f32, i64)`: 元组包含补偿后的温度值(°C)和 t_fine 值
    ///
    /// **算法特点**
    /// - 使用二阶多项式补偿温度传感器的非线性响应
    /// - 生成高精度中间值 t_fine 用于后续计算
    /// - 提供 0.01°C 的分辨率
    ///
    /// **精度指标**
    /// - 分辨率: 0.01°C
    /// - 绝对精度: ±0.5°C (0-65°C范围内)
    /// - 长期稳定性: ±0.08°C/年
    fn compensate_temperature(&self, adc_t: i32) -> (f32, i64) {
        // 提取温度补偿数据编译换算（注意温度补偿运算是在32位有符号整型下转换的）
        let dig_t1 = self.calib.dig_t1 as i32;
        let dig_t2 = self.calib.dig_t2 as i32;
        let dig_t3 = self.calib.dig_t3 as i32;
        // 带入公式进行换算
        let var1 = (((adc_t >> 3) - (dig_t1 << 1)) * dig_t2) >> 11;
        let var2 = ((((adc_t >> 4) - dig_t1) * ((adc_t >> 4) - dig_t1)) >> 12) * dig_t3;
        let var2 = var2 >> 14;

        // 计算中间变量(后面的压力转换和湿度转换需要依赖温度的变化做补偿)
        let t_fine = (var1 as i64) + (var2 as i64);
        // 换算位摄氏度
        let temperature = (t_fine * 5 + 128) >> 8; // in 0.01°C

        // OK
        ((temperature as f64 / 100.0) as f32, t_fine)
    }

    /// BME280 压力补偿函数
    ///
    /// **功能描述**
    /// 根据数据手册 4.2.3 节的压力补偿公式，将原始 ADC 压力值转换为
    /// 以帕斯卡(Pa)为单位的压力值，使用温度补偿生成的 t_fine 值。
    ///
    /// **参数**
    /// - `adc_p`: 从寄存器 0xF7-0xF9 读取的原始20位压力ADC值
    /// - `t_fine`: 从温度补偿计算得到的高精度温度中间值
    ///
    /// **返回**
    /// - `f32`: 补偿后的压力值(Pa)
    ///
    /// **算法特点**
    /// - 使用复杂的多项式补偿压力传感器的非线性响应
    /// - 包含温度依赖性补偿和灵敏度校正
    /// - 提供 0.18Pa 的分辨率
    ///
    /// **精度指标**
    /// - 分辨率: 0.18Pa (相当于1.7cm高度)
    /// - 绝对精度: ±1.0hPa (300-1100hPa, 0-65°C)
    /// - 温度系数: ±1.5Pa/K
    fn compensate_pressure(&self, adc_p: i32, t_fine: i64) -> f32 {
        // 提取压力补偿数据编译换算（注意压力补偿运算是在64位有符号整型下转换的）
        let dig_p1 = self.calib.dig_p1 as i64;
        let dig_p2 = self.calib.dig_p2 as i64;
        let dig_p3 = self.calib.dig_p3 as i64;
        let dig_p4 = self.calib.dig_p4 as i64;
        let dig_p5 = self.calib.dig_p5 as i64;
        let dig_p6 = self.calib.dig_p6 as i64;
        let dig_p7 = self.calib.dig_p7 as i64;
        let dig_p8 = self.calib.dig_p8 as i64;
        let dig_p9 = self.calib.dig_p9 as i64;

        // 步骤1: 计算温度相关变量
        // var1 = t_fine - 128000
        let mut var1 = t_fine - 128000;

        // 步骤2: 计算二阶补偿项
        // var2 = var1 * var1 * dig_P6
        let mut var2 = var1 * var1 * dig_p6;
        // var2 = var2 + (var1 * dig_P5 << 17)
        var2 = var2 + ((var1 * dig_p5) << 17);
        // var2 = var2 + (dig_P4 << 35)
        var2 = var2 + (dig_p4 << 35);

        // 步骤3: 计算主补偿项
        // var1 = ((var1 * var1 * dig_P3) >> 8) + ((var1 * dig_P2) << 12)
        var1 = ((var1 * var1 * dig_p3) >> 8) + ((var1 * dig_p2) << 12);
        // var1 = (((1 << 47) + var1) * dig_P1) >> 33
        var1 = ((((1_i64) << 47) + var1) * dig_p1) >> 33;

        // 步骤4: 检查除零错误
        // 避免因除零导致的异常
        if var1 == 0 {
            return 0.0;
        }

        // 步骤5: 计算初步压力值
        // p = 1048576 - adc_p
        let mut p = 1048576 - (adc_p as i64);
        // p = ((p << 31) - var2) * 3125 / var1
        p = (((p << 31) - var2) * 3125) / var1;

        // 步骤6: 应用最终补偿
        // var1 = (dig_P9 * (p>>13) * (p>>13)) >> 25
        var1 = (dig_p9 * ((p >> 13) * (p >> 13))) >> 25;
        // var2 = (dig_P8 * p) >> 19
        var2 = (dig_p8 * p) >> 19;
        // p = ((p + var1 + var2) >> 8) + (dig_P7 << 4)
        p = ((p + var1 + var2) >> 8) + (dig_p7 << 4);

        // 返回压力值
        (p as f64 / 256.0) as f32
    }

    /// 补偿湿度数据 - 修正版本
    ///
    /// **算法说明**
    /// 根据数据手册 4.2.3 节的湿度补偿公式实现
    /// 使用分步计算提高可读性和可靠性
    ///
    /// **参数**
    /// - `adc_h`: 从寄存器 0xFD-0xFE 读取的原始16位湿度ADC值
    ///
    /// **返回**
    /// - `f32`: 补偿后的湿度值(%RH)，范围 0.0-100.0
    fn compensate_humidity(&self, adc_h: i32, t_fine: i64) -> f32 {
        // 提取湿度补偿数据编译换算（注意湿度补偿运算是在32位有符号整型下转换的）
        let dig_h1 = self.calib.dig_h1 as i32;
        let dig_h2 = self.calib.dig_h2 as i32;
        let dig_h3 = self.calib.dig_h3 as i32;
        let dig_h4 = self.calib.dig_h4 as i32;
        let dig_h5 = self.calib.dig_h5 as i32;
        let dig_h6 = self.calib.dig_h6 as i32;

        // 步骤1: 计算温度调整项
        // var1 = t_fine - 76800
        let var1 = (t_fine - 76800) as i32;

        // 步骤2: 复杂的主补偿计算
        let var2 = (((adc_h << 14) - (dig_h4 << 20) - (dig_h5 * var1)) + 16384) >> 15;
        let var3 = (((var1 * dig_h6) >> 10) * (((var1 * dig_h3) >> 11) + 32768)) >> 10;
        let var4 = ((var3 + 2097152) * dig_h2 + 8192) >> 14;
        let mut var5 = var2 * var4;

        // 步骤3: 非线性补偿
        var5 = var5 - (((((var5 >> 15) * (var5 >> 15)) >> 7) * dig_h1) >> 4);

        // 步骤4: 限制输出范围
        var5 = if var5 < 0 { 0 } else { var5 };
        var5 = if var5 > 419430400 { 419430400 } else { var5 };

        // 返回相对湿度: Q22.10格式的湿度值 / 1024
        (((var5 >> 12) as u32) / 1024) as f32
    }

    /// Read BME280 sensor data
    pub fn read<B: I2c<SevenBitAddress>>(
        &mut self,
        bus: &mut B,
    ) -> Result<(f32, f32, f32), B::Error> {
        // 读取原始数据
        let (adc_p, adc_t, adc_h) = self.read_raw_data(bus)?;

        // 使用补偿公式补偿数据
        let (temperature, t_fine) = self.compensate_temperature(adc_t);
        let pressure = self.compensate_pressure(adc_p, t_fine);
        let humidity = self.compensate_humidity(adc_h, t_fine);

        // OK
        Ok((temperature, pressure, humidity))
    }

    /// Soft reset sensor
    pub fn reset<B: I2c<SevenBitAddress>>(&mut self, bus: &mut B) -> Result<(), B::Error> {
        // 软重置
        bus.write(self.address, &[0xE0, 0xB6])?;
        // 等待重置完成
        self.delay_impl.delay(Duration::from_millis(5));
        // 重新读取校准数据
        self.calib = Self::read_calibration_data(bus, self.address)?;
        // OK
        Ok(())
    }
}
