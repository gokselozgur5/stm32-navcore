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
    Odr2kHz = 0x05,
    Odr1kHz = 0x06,
    Odr200Hz = 0x07,
    Odr100Hz = 0x08,
    Odr50Hz = 0x09,
    Odr25Hz = 0x0A,
    Odr12_5Hz = 0x0B,
}

/// Anti-Aliasing Filter (AAF) configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AafConfig {
    pub delt: u8,
    pub deltsqr: u16,
    pub bitshift: u8,
}

impl AafConfig {
    /// Pre-configured 258 Hz cutoff 2nd order AAF (1 kHz ODR).
    pub const fn cutoff_258hz() -> Self {
        Self {
            delt: 6,
            deltsqr: 36,
            bitshift: 10,
        }
    }

    /// Pre-configured 536 Hz cutoff 2nd order AAF (2 kHz ODR).
    pub const fn cutoff_536hz() -> Self {
        Self {
            delt: 13,
            deltsqr: 169,
            bitshift: 8,
        }
    }
}

/// Raw unscaled sensor sample.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RawSample {
    pub accel: [i16; 3],
    pub gyro: [i16; 3],
    pub temp: i16,
}

/// Calibrated physical unit sample (m/s^2, rad/s, deg C).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ScaledSample {
    pub accel_m_s2: [f32; 3],
    pub gyro_rad_s: [f32; 3],
    pub temp_c: f32,
}

/// ICM-42688-P driver state and configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Icm42688 {
    pub gyro_fsr: GyroFsr,
    pub accel_fsr: AccelFsr,
    pub odr: Odr,
    pub aaf: Option<AafConfig>,
}

impl Default for Icm42688 {
    fn default() -> Self {
        Self::new()
    }
}

impl Icm42688 {
    /// Initialize with default 2000 dps, 16 g, 1 kHz ODR, and 258 Hz AAF.
    pub const fn new() -> Self {
        Self {
            gyro_fsr: GyroFsr::Dps2000,
            accel_fsr: AccelFsr::G16,
            odr: Odr::Odr1kHz,
            aaf: Some(AafConfig::cutoff_258hz()),
        }
    }
}

impl Icm42688 {
    /// Gyro sensitivity in (rad/s) per LSB.
    pub fn gyro_sensitivity_rad_s(&self) -> f32 {
        let dps = match self.gyro_fsr {
            GyroFsr::Dps2000 => 2000.0,
            GyroFsr::Dps1000 => 1000.0,
            GyroFsr::Dps500 => 500.0,
            GyroFsr::Dps250 => 250.0,
            GyroFsr::Dps125 => 125.0,
            GyroFsr::Dps62_5 => 62.5,
            GyroFsr::Dps31_25 => 31.25,
            GyroFsr::Dps15_625 => 15.625,
        };
        (dps / 32768.0) * (core::f32::consts::PI / 180.0)
    }
}

impl Icm42688 {
    /// Accel sensitivity in (m/s^2) per LSB.
    pub fn accel_sensitivity_m_s2(&self) -> f32 {
        let g = match self.accel_fsr {
            AccelFsr::G16 => 16.0,
            AccelFsr::G8 => 8.0,
            AccelFsr::G4 => 4.0,
            AccelFsr::G2 => 2.0,
        };
        (g / 32768.0) * 9.80665
    }
}

impl Icm42688 {
    /// Convert raw 16-bit temperature register value to degrees Celsius.
    pub fn scale_temperature(raw_temp: i16) -> f32 {
        (raw_temp as f32 / 132.48) + 25.0
    }
}

impl Icm42688 {
    /// Scale raw sample into SI engineering units (m/s^2, rad/s, C).
    pub fn scale_sample(&self, raw: &RawSample) -> ScaledSample {
        let g_sens = self.gyro_sensitivity_rad_s();
        let a_sens = self.accel_sensitivity_m_s2();
        ScaledSample {
            accel_m_s2: [
                raw.accel[0] as f32 * a_sens,
                raw.accel[1] as f32 * a_sens,
                raw.accel[2] as f32 * a_sens,
            ],
            gyro_rad_s: [
                raw.gyro[0] as f32 * g_sens,
                raw.gyro[1] as f32 * g_sens,
                raw.gyro[2] as f32 * g_sens,
            ],
            temp_c: Self::scale_temperature(raw.temp),
        }
    }
}

impl Icm42688 {
    /// Encode SPI write transaction buffer (write flag = 0x00).
    pub fn encode_write_reg(reg: u8, val: u8) -> [u8; 2] {
        [reg & 0x7F, val]
    }

    /// Encode SPI read transaction buffer (read flag = 0x80).
    pub fn encode_read_reg(reg: u8) -> [u8; 2] {
        [reg | 0x80, 0x00]
    }
}

impl Icm42688 {
    /// Encode GyroConfig0 register value.
    pub fn encode_gyro_config0(&self) -> u8 {
        ((self.gyro_fsr as u8) << 5) | (self.odr as u8)
    }

    /// Encode AccelConfig0 register value.
    pub fn encode_accel_config0(&self) -> u8 {
        ((self.accel_fsr as u8) << 5) | (self.odr as u8)
    }
}

impl Icm42688 {
    /// Encode Gyro Anti-Aliasing Filter (AAF) registers (STATIC3, 4, 5).
    pub fn encode_gyro_aaf_regs(&self) -> Option<[[u8; 2]; 3]> {
        self.aaf.map(|aaf| {
            [
                [aaf_reg::GYRO_CONFIG_STATIC3, aaf.delt],
                [aaf_reg::GYRO_CONFIG_STATIC4, (aaf.deltsqr & 0xFF) as u8],
                [
                    aaf_reg::GYRO_CONFIG_STATIC5,
                    (((aaf.deltsqr >> 8) & 0x0F) as u8) | (aaf.bitshift << 4),
                ],
            ]
        })
    }
}

impl Icm42688 {
    /// Encode Accel Anti-Aliasing Filter (AAF) registers (STATIC2, 3, 4).
    pub fn encode_accel_aaf_regs(&self) -> Option<[[u8; 2]; 3]> {
        self.aaf.map(|aaf| {
            [
                [aaf_reg::ACCEL_CONFIG_STATIC2, aaf.delt << 1],
                [aaf_reg::ACCEL_CONFIG_STATIC3, (aaf.deltsqr & 0xFF) as u8],
                [
                    aaf_reg::ACCEL_CONFIG_STATIC4,
                    (((aaf.deltsqr >> 8) & 0x0F) as u8) | (aaf.bitshift << 4),
                ],
            ]
        })
    }
}

impl Icm42688 {
    /// Parse 16-bit packet from FIFO (header, accel[3], gyro[3], temp).
    pub fn parse_fifo_packet_16bit(buf: &[u8]) -> Option<RawSample> {
        if buf.len() < 16 {
            return None;
        }
        let header = buf[0];
        if header & 0x80 != 0 {
            return None;
        }
        let accel = [
            i16::from_be_bytes([buf[1], buf[2]]),
            i16::from_be_bytes([buf[3], buf[4]]),
            i16::from_be_bytes([buf[5], buf[6]]),
        ];
        let gyro = [
            i16::from_be_bytes([buf[7], buf[8]]),
            i16::from_be_bytes([buf[9], buf[10]]),
            i16::from_be_bytes([buf[11], buf[12]]),
        ];
        let temp = i16::from_be_bytes([buf[13], buf[14]]);
        Some(RawSample { accel, gyro, temp })
    }
}

/// High-resolution 20-bit raw sample decoded from extended FIFO format.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct RawSample20Bit {
    pub accel_20bit: [i32; 3],
    pub gyro_20bit: [i32; 3],
    pub temp: i16,
}

impl Icm42688 {
    fn extract_20bit(msb: i16, lsb_nibble: u8) -> i32 {
        ((msb as i32) << 4) | (lsb_nibble as i32 & 0x0F)
    }
}

impl Icm42688 {
    /// Parse 20-bit packet from extended FIFO (20 bytes).
    pub fn parse_fifo_packet_20bit(buf: &[u8]) -> Option<RawSample20Bit> {
        if buf.len() < 20 {
            return None;
        }
        let a = [
            i16::from_be_bytes([buf[1], buf[2]]),
            i16::from_be_bytes([buf[3], buf[4]]),
            i16::from_be_bytes([buf[5], buf[6]]),
        ];
        let g = [
            i16::from_be_bytes([buf[7], buf[8]]),
            i16::from_be_bytes([buf[9], buf[10]]),
            i16::from_be_bytes([buf[11], buf[12]]),
        ];
        let temp = i16::from_be_bytes([buf[13], buf[14]]);
        let (e1, e2, e3) = (buf[15], buf[16], buf[17]);
        let accel_20bit = [Self::extract_20bit(a[0], e1 >> 4), Self::extract_20bit(a[1], e1), Self::extract_20bit(a[2], e2 >> 4)];
        let gyro_20bit = [Self::extract_20bit(g[0], e2), Self::extract_20bit(g[1], e3 >> 4), Self::extract_20bit(g[2], e3)];
        Some(RawSample20Bit { accel_20bit, gyro_20bit, temp })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_icm42688_default_construction_and_sensitivities() {
        let imu = Icm42688::new();
        assert_eq!(imu.gyro_fsr, GyroFsr::Dps2000);
        assert_eq!(imu.accel_fsr, AccelFsr::G16);
        assert!((imu.gyro_sensitivity_rad_s() - 0.00106526).abs() < 1e-6);
        assert!((imu.accel_sensitivity_m_s2() - 0.0047884).abs() < 1e-6);
    }

    #[test]
    fn test_icm42688_register_encoding_and_aaf() {
        let imu = Icm42688::new();
        assert_eq!(Icm42688::encode_write_reg(reg::PWR_MGMT0, 0x0F), [0x4E, 0x0F]);
        assert_eq!(Icm42688::encode_read_reg(reg::WHO_AM_I), [0xF5, 0x00]);
        assert_eq!(imu.encode_gyro_config0(), 0x06);
        assert_eq!(imu.encode_accel_config0(), 0x06);
        let aaf = imu.encode_gyro_aaf_regs().unwrap();
        assert_eq!(aaf[0], [0x0C, 6]);
        assert_eq!(aaf[1], [0x0D, 36]);
    }

    #[test]
    fn test_icm42688_fifo_packet_decoding() {
        let mut pkt = [0u8; 20];
        pkt[0] = 0x00;
        pkt[1..3].copy_from_slice(&1000i16.to_be_bytes());
        pkt[7..9].copy_from_slice(&(-500i16).to_be_bytes());
        pkt[13..15].copy_from_slice(&0i16.to_be_bytes());
        let raw16 = Icm42688::parse_fifo_packet_16bit(&pkt).unwrap();
        assert_eq!(raw16.accel[0], 1000);
        assert_eq!(raw16.gyro[0], -500);
        assert_eq!(Icm42688::scale_temperature(raw16.temp), 25.0);
    }

    #[test]
    fn test_icm42688_sample_scaling_and_rejection() {
        let imu = Icm42688::new();
        let raw = RawSample { accel: [0, 0, 2046], gyro: [0, 0, 0], temp: 0 };
        let scaled = imu.scale_sample(&raw);
        assert!((scaled.accel_m_s2[2] - 9.8).abs() < 0.1);
        assert!(Icm42688::parse_fifo_packet_16bit(&[0x80; 16]).is_none());
        assert!(Icm42688::parse_fifo_packet_16bit(&[0u8; 10]).is_none());
    }
}
