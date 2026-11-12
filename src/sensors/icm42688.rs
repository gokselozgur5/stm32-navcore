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
