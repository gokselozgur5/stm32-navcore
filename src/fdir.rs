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
        }
    }

    /// Return the currently active IMU.
    pub fn active_imu(&self) -> ImuId {
        self.active_imu
    }

    /// Return the health status of the primary IMU.
    pub fn primary_health(&self) -> ImuHealth {
        self.primary_health
    }

    /// Return the health status of the secondary IMU.
    pub fn secondary_health(&self) -> ImuHealth {
        self.secondary_health
    }

    /// Total failovers executed.
    pub fn total_failovers(&self) -> u32 {
        self.total_failovers
    }

    fn vector_diff_norm_sq(a: &[f32; 3], b: &[f32; 3]) -> f32 {
        let dx = a[0] - b[0];
        let dy = a[1] - b[1];
        let dz = a[2] - b[2];
        dx * dx + dy * dy + dz * dz
    }

    /// Compute normalized chi-square statistic for gyro difference.
    pub fn compute_gyro_chi_square(&self, s1: &ImuSample, s2: &ImuSample, var: f32) -> f32 {
        if var <= 1e-9 {
            return 0.0;
        }
        let norm_sq = Self::vector_diff_norm_sq(&s1.gyro_rads, &s2.gyro_rads);
        norm_sq / var
    }

    /// Process an epoch with samples from primary and secondary IMUs.
    pub fn process_epoch(
        &mut self,
        primary: &ImuSample,
        secondary: &ImuSample,
        dt_s: f32,
        ekf: &mut NavEkf,
    ) -> Result<ImuSample, EkfError> {
        let gyro_diff_sq = Self::vector_diff_norm_sq(&primary.gyro_rads, &secondary.gyro_rads);
        let accel_diff_sq = Self::vector_diff_norm_sq(&primary.accel_mps2, &secondary.accel_mps2);

        let gyro_thresh_sq = self.config.gyro_discrepancy_threshold_rads
            * self.config.gyro_discrepancy_threshold_rads;
        let accel_thresh_sq = self.config.accel_discrepancy_threshold_mps2
            * self.config.accel_discrepancy_threshold_mps2;

        let has_discrepancy = gyro_diff_sq > gyro_thresh_sq || accel_diff_sq > accel_thresh_sq;

        if has_discrepancy {
            self.consecutive_discrepancies = self.consecutive_discrepancies.saturating_add(1);
            if self.consecutive_discrepancies >= self.config.consecutive_fault_limit {
                if self.active_imu == ImuId::Primary && self.secondary_health != ImuHealth::Faulted {
                    self.primary_health = ImuHealth::Faulted;
                    self.active_imu = ImuId::Secondary;
                    self.total_failovers = self.total_failovers.saturating_add(1);
                } else if self.active_imu == ImuId::Secondary && self.primary_health != ImuHealth::Faulted {
                    self.secondary_health = ImuHealth::Faulted;
