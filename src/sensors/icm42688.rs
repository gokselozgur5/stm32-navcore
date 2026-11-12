//! InvenSense ICM-42688-P 6-axis ultra-low noise IMU driver.

/// ICM-42688-P SPI register addresses.
pub mod reg {
    pub const DEVICE_CONFIG: u8 = 0x11;
    pub const DRIVE_CONFIG: u8 = 0x13;
    pub const INT_CONFIG: u8 = 0x14;
    pub const FIFO_COUNTH: u8 = 0x2E;
    pub const FIFO_COUNTL: u8 = 0x2F;
    pub const FIFO_DATA: u8 = 0x30;
    pub const INT_STATUS: u8 = 0x2D;
    pub const ACCEL_CONFIG0: u8 = 0x50;
    pub const GYRO_CONFIG0: u8 = 0x4F;
    pub const GYRO_ACCEL_CONFIG0: u8 = 0x52;
    pub const ACCEL_CONFIG1: u8 = 0x53;
    pub const PWR_MGMT0: u8 = 0x4E;
    pub const WHO_AM_I: u8 = 0x75;
}

pub mod aaf_reg {
    pub const GYRO_CONFIG_STATIC3: u8 = 0x0C;
    pub const GYRO_CONFIG_STATIC4: u8 = 0x0D;
    pub const GYRO_CONFIG_STATIC5: u8 = 0x0E;
    pub const ACCEL_CONFIG_STATIC2: u8 = 0x03;
    pub const ACCEL_CONFIG_STATIC3: u8 = 0x04;
    pub const ACCEL_CONFIG_STATIC4: u8 = 0x05;
}

/// Gyro full-scale range in degrees per second.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum GyroFsr {
    Dps2000 = 0x00,
    Dps1000 = 0x01,
    Dps500 = 0x02,
    Dps250 = 0x03,
    Dps125 = 0x04,
    Dps62_5 = 0x05,
    Dps31_25 = 0x06,
    Dps15_625 = 0x07,
}

/// Accel full-scale range in g.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AccelFsr {
    G16 = 0x00,
    G8 = 0x01,
    G4 = 0x02,
    G2 = 0x03,
}

/// Output Data Rate (ODR) for Gyro and Accel.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Odr {
    Odr32kHz = 0x01,
    Odr16kHz = 0x02,
    Odr8kHz = 0x03,
    Odr4kHz = 0x04,
