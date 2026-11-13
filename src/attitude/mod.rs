//! Attitude determination algorithms.

pub mod error;
pub mod madgwick;
pub mod mahony;

pub use error::{attitude_error, attitude_error_angle, RelativeAttitude};
pub use madgwick::Madgwick;
pub use mahony::Mahony;

/// Errors arising during attitude estimation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AttitudeError {
    InvalidInput,
    NonFinite,
}
