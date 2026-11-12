//! Bosch BMI088 dual-die aerospace 6-DoF IMU driver and register codec.

/// Expected chip ID for BMI088 accelerometer.
pub const BMI088_ACCEL_CHIP_ID: u8 = 0x1E;

/// Expected chip ID for BMI088 gyroscope.
pub const BMI088_GYRO_CHIP_ID: u8 = 0x0F;

/// Errors arising during BMI088 register operations.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bmi088Error {
    InvalidChipId { expected: u8, actual: u8 },
    CommunicationError,
    SelfTestFailed,
    DataNotReady,
}

/// BMI088 Accelerometer SPI registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelRegister {
    ChipId = 0x00,
    ErrReg = 0x02,
    Status = 0x03,
    XRateL = 0x12,
    XRateH = 0x13,
    YRateL = 0x14,
    YRateH = 0x15,
    ZRateL = 0x16,
    ZRateH = 0x17,
    Sensortime0 = 0x18,
    IntStat1 = 0x1D,
    AccelConf = 0x40,
    AccelRange = 0x41,
    Int1IoConf = 0x53,
    Int2IoConf = 0x54,
    IntMapData = 0x58,
    AccelPwrConf = 0x7C,
    AccelPwrCtr = 0x7D,
    Softreset = 0x7E,
}

/// BMI088 Gyroscope SPI registers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GyroRegister {
    ChipId = 0x00,
    RateXlsb = 0x02,
    RateXmsb = 0x03,
    RateYlsb = 0x04,
    RateYmsb = 0x05,
    RateZlsb = 0x06,
    RateZmsb = 0x07,
    IntStat1 = 0x0A,
    Range = 0x0F,
    Bw = 0x10,
    Lpm1 = 0x11,
    Softreset = 0x14,
    IntCtrl = 0x15,
    Int34IoConf = 0x16,
    Int34IoMap = 0x18,
    SelfTest = 0x3C,
}

/// BMI088 Accelerometer measurement range selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelRange {
    Range3g = 0x00,
    Range6g = 0x01,
    Range12g = 0x02,
    Range24g = 0x03,
}

impl AccelRange {
    /// Return full-scale range in units of standard gravity (g).
    pub const fn max_g(&self) -> f32 {
        match self {
            Self::Range3g => 3.0,
            Self::Range6g => 6.0,
            Self::Range12g => 12.0,
            Self::Range24g => 24.0,
        }
    }
}

impl AccelRange {
    /// Convert raw 16-bit signed integer reading into m/s^2.
    pub fn scale_m_s2(&self, raw: i16) -> f32 {
        let max_g = self.max_g();
        let g_val = (raw as f32 / 32768.0) * max_g;
        g_val * 9.80665
    }
}

/// BMI088 Accelerometer Output Data Rate (ODR) and filter bandwidth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelOdr {
    Odr12_5Hz = 0x05,
    Odr25Hz = 0x06,
    Odr50Hz = 0x07,
    Odr100Hz = 0x08,
    Odr200Hz = 0x09,
    Odr400Hz = 0x0A,
    Odr800Hz = 0x0B,
    Odr1600Hz = 0x0C,
}

/// BMI088 Gyroscope full-scale measurement range.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GyroRange {
    Range2000Dps = 0x00,
    Range1000Dps = 0x01,
    Range500Dps = 0x02,
    Range250Dps = 0x03,
    Range125Dps = 0x04,
}

impl GyroRange {
    /// Return full-scale range in degrees per second (deg/s).
    pub const fn max_dps(&self) -> f32 {
        match self {
            Self::Range2000Dps => 2000.0,
            Self::Range1000Dps => 1000.0,
            Self::Range500Dps => 500.0,
            Self::Range250Dps => 250.0,
            Self::Range125Dps => 125.0,
        }
    }
}
