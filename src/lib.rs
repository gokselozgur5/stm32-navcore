//! Deterministic `no_std`, zero-alloc navigation core for STM32 Cortex-M4F.
//!
//! Modules: [`math`] (matrices, SO(3) quaternion, tilt-compensated heading),
//! [`ekf`] (Joseph-form IMU/GNSS EKF) and [`protocol`] (COBS, CRC-16, CAN 2.0B).
#![cfg_attr(not(test), no_std)]

pub mod ekf;
pub use ekf::StationaryDetector;
pub mod math;
pub mod protocol;

#[cfg(test)]
mod footprint {
    use crate::ekf::NavEkf;

    #[test]
    fn nav_ekf_fits_sram_budget() {
        assert!(core::mem::size_of::<NavEkf>() < 2048);
    }
}
