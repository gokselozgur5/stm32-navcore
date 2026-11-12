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
#[derive(Debug, Clone, Copy)]
pub struct DualImuConfig {
    /// Maximum allowed specific force difference before trip (m/s^2)
    pub accel_discrepancy_threshold_mps2: f32,
    /// Maximum allowed angular rate difference before trip (rad/s)
    pub gyro_discrepancy_threshold_rads: f32,
    /// Chi-square statistic trip threshold for 3-DoF difference
    pub chi_square_threshold: f32,
    /// Number of consecutive faulty epochs required to isolate
    pub consecutive_fault_limit: u8,
}

impl Default for DualImuConfig {
    fn default() -> Self {
        Self {
            accel_discrepancy_threshold_mps2: 2.5,
            gyro_discrepancy_threshold_rads: 0.15,
            chi_square_threshold: 7.815,
            consecutive_fault_limit: 3,
        }
    }
}

/// Dual IMU FDIR Manager.
#[derive(Debug, Clone)]
pub struct DualImuManager {
    config: DualImuConfig,
    active_imu: ImuId,
    primary_health: ImuHealth,
    secondary_health: ImuHealth,
    consecutive_discrepancies: u8,
    total_failovers: u32,
}

impl DualImuManager {
    /// Create a new DualImuManager with given configuration.
    pub fn new(config: DualImuConfig) -> Self {
        Self {
            config,
            active_imu: ImuId::Primary,
            primary_health: ImuHealth::Nominal,
            secondary_health: ImuHealth::Nominal,
            consecutive_discrepancies: 0,
            total_failovers: 0,
