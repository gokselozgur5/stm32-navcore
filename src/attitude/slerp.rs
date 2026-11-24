//! Attitude interpolation delegating to canonical rotation slerp/nlerp.

pub use crate::math::rotation::slerp::{nlerp as attitude_nlerp, slerp as attitude_slerp};
