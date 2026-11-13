//! Rotation utilities built on the `math` types.

pub mod dcm;
pub mod quat_dcm;
pub mod slerp;
pub mod so3_exp;

pub use dcm::{Dcm, RotationError};
