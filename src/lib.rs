//! Deterministic `no_std`, zero-alloc navigation core for STM32 Cortex-M4F.
//!
//! Modules:
//!
//! - [`math`]: fixed-size matrices, SO(3) quaternion, tilt-compensated heading.
//! - [`ekf`]: Joseph-form IMU/GNSS extended Kalman filter.
//! - [`fdir`]: dual-IMU fault detection and switchover.
//! - [`sensors`]: flight sensor drivers and register codecs.
//! - [`protocol`]: COBS, CRC-16-CCITT, CAN 2.0B and UART telemetry framing.
//! - [`ubx`]: streaming u-blox UBX parser.
//!
//! See `docs/architecture.md` in the repository for the data flow between modules.
#![cfg_attr(not(test), no_std)]

pub mod attitude;
pub mod ekf;
/// Zero-velocity detector over a sliding window of accelerometer and gyro samples.
pub use ekf::StationaryDetector;
pub mod fdir;
pub mod filters;
pub mod math;
pub mod protocol;
pub mod sensors;
pub mod ubx;

/// Dual-IMU fault detection types: IMU identity, a sample, and the switchover manager.
pub use fdir::{DualImuManager, ImuId, ImuReading};

/// Versioned, checksummed magnetometer and gyro calibration record.
pub use protocol::CalibrationBlock;
/// Decoded u-blox UBX messages and the streaming parser that produces them.
pub use ubx::{UbxMessage, UbxNavPvt, UbxNavRelPosNed, UbxParser};

#[cfg(test)]
mod footprint {
    use crate::ekf::NavEkf;

    #[test]
    fn nav_ekf_fits_sram_budget() {
        assert!(core::mem::size_of::<NavEkf>() < 2048);
    }
}
