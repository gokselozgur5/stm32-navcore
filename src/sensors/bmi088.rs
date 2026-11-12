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
