//! Attitude determination algorithms.

pub mod error;
pub mod madgwick;
pub mod mahony;
pub mod slerp;

pub use error::{attitude_error, attitude_error_angle, RelativeAttitude};
pub use madgwick::Madgwick;
pub use mahony::Mahony;
pub use slerp::{attitude_nlerp, attitude_slerp};

/// Errors arising during attitude estimation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttitudeError {
    InvalidInput,
    NonFinite,
}
