//! Attitude determination algorithms.

pub mod cov;
pub mod error;
pub mod gyro_bias;
pub mod madgwick;
pub mod madgwick_ahrs;
pub mod mahony;
pub mod mahony_ahrs;
pub mod slerp;

pub use cov::propagate_attitude_cov;
pub use error::{attitude_error, attitude_error_angle, RelativeAttitude};
pub use gyro_bias::{
    apply_bias_slew_limit, clamp_bias_vector, correct_gyro_sample, track_bias_from_error,
    track_bias_stationary, GyroBiasConfig, GyroBiasError, GyroBiasTracker,
};
pub use madgwick::Madgwick;
pub use madgwick_ahrs::{madgwick_imu_step, madgwick_marg_step, MadgwickAhrs, MadgwickAhrsError};
pub use mahony::Mahony;
pub use mahony_ahrs::{
    mahony_ahrs_6dof_step, mahony_ahrs_9dof_step, MahonyAhrs, MahonyAhrsError, MahonyInputs,
};
pub use slerp::{attitude_nlerp, attitude_slerp};

/// Errors arising during attitude estimation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttitudeError {
    InvalidInput,
    NonFinite,
}
