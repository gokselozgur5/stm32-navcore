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
