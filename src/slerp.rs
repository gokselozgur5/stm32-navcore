//! Re-exports of quaternion spherical linear and normalized linear interpolation.

pub use crate::rotation::slerp::{
    dot as quat_dot, nlerp, nlerp as quat_nlerp, slerp, slerp as quat_slerp,
};
