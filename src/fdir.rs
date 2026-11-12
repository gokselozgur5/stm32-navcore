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

/// Identifier for redundant IMU sources.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuId {
    /// Primary IMU (e.g. BMI088 or primary slot)
    Primary = 0,
    /// Secondary redundant IMU (e.g. ICM-42688-P or secondary slot)
    Secondary = 1,
}

/// Operational health state of an individual IMU.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImuHealth {
    /// Sensor is healthy and within statistical bounds
    Nominal,
    /// Parity / innovation discrepancy observed but below trip count
    Suspect(u8),
    /// Sensor has suffered hard failure or chi-square trip; isolated
    Faulted,
}

/// Configuration thresholds for dual-IMU FDIR.
