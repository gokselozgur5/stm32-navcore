//! Dual-IMU Sensor Redundancy, Voting & Chi-Square Fault Isolation (FDIR).
//!
//! Provides deterministic `#![no_std]`, zero-heap fault detection, isolation,
//! and recovery across redundant primary and secondary 6-DoF IMUs.

use crate::ekf::{EkfError, NavEkf};

/// Strongly-typed 6-DoF IMU sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    pub accel_mps2: [f32; 3],
    pub gyro_rads: [f32; 3],
}

impl ImuSample {
    pub const fn new(accel_mps2: [f32; 3], gyro_rads: [f32; 3]) -> Self {
        Self {
            accel_mps2,
            gyro_rads,
        }
    }
}
