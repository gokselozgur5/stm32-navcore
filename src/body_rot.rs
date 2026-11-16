//! NED to Body frame rotation matrix operations.

use crate::math::Mat;
use libm::{asinf, atan2f, cosf, sinf, sqrtf};

/// Error types for NED to Body rotations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BodyRotError {
    /// Non-finite coordinate, vector, or matrix component.
    NonFinite,
    /// Matrix failed orthonormality verification.
    NotOrthonormal,
    /// Gimbal lock condition in Euler conversion.
    GimbalLock,
}

/// Numerical tolerance for DCM orthonormality checks.
pub const BODY_ROT_TOLERANCE: f32 = 1e-4;

/// Rotation representation between NED and Body coordinate frames.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyRot {
    pub(crate) dcm_nb: Mat<3, 3>,
}

impl BodyRot {
    /// Creates an identity rotation where NED and Body frames align.
    pub fn identity() -> Self {
        Self {
            dcm_nb: Mat::identity(),
        }
    }

    /// Returns a reference to the NED-to-Body DCM (C_n_b).
    pub fn dcm_ned_to_body(&self) -> &Mat<3, 3> {
        &self.dcm_nb
    }

    /// Returns the transposed Body-to-NED DCM (C_b_n).
    pub fn dcm_body_to_ned(&self) -> Mat<3, 3> {
        self.dcm_nb.transpose()
    }
}
