//! Rotation utilities built on the `math` types.

pub mod attitude_error;
pub mod dcm;
pub mod orthonormalize;
pub mod quat_dcm;
pub mod slerp;
pub mod so3_exp;

pub use attitude_error::{attitude_error, AttitudeError};
pub use dcm::{Dcm, RotationError};
