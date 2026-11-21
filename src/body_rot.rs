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
    pub dcm_nb: Mat<3, 3>,
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

impl BodyRot {
    /// Computes determinant of the 3x3 rotation matrix.
    pub fn det(&self) -> f32 {
        let m = &self.dcm_nb.0;
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    }

    /// Checks if inner DCM satisfies orthonormality.
    pub fn is_orthonormal(&self) -> bool {
        let p = self.dcm_nb.matmul(&self.dcm_nb.transpose());
        for i in 0..3 {
            for j in 0..3 {
                let target = if i == j { 1.0 } else { 0.0 };
                if (p.0[i][j] - target).abs() > BODY_ROT_TOLERANCE {
                    return false;
                }
            }
        }
        (self.det() - 1.0).abs() <= BODY_ROT_TOLERANCE
    }
}

impl BodyRot {
    /// Builds a rotation from an existing NED-to-Body DCM.
    pub fn from_dcm_ned_to_body(m: Mat<3, 3>) -> Result<Self, BodyRotError> {
        crate::rotation::Dcm::from_dcm_ned_to_body(m).map(|d| Self { dcm_nb: d.0 })
    }

    /// Builds a rotation from an existing Body-to-NED DCM.
    pub fn from_dcm_body_to_ned(m: Mat<3, 3>) -> Result<Self, BodyRotError> {
        crate::rotation::Dcm::from_dcm_body_to_ned(m).map(|d| Self { dcm_nb: d.0 })
    }

    /// Builds NED-to-Body rotation from Euler angles (roll, pitch, yaw).
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Result<Self, BodyRotError> {
        crate::rotation::Dcm::from_euler(roll, pitch, yaw).map(|d| Self { dcm_nb: d.0 })
    }

    /// Extracts Euler angles (roll, pitch, yaw) in radians.
    pub fn to_euler(&self) -> Result<(f32, f32, f32), BodyRotError> {
        crate::rotation::Dcm(self.dcm_nb).to_euler()
    }

    /// Rotates vector from NED frame to Body frame: v_b = C_n_b * v_n.
    pub fn rotate_ned_to_body(&self, v: [f32; 3]) -> Result<[f32; 3], BodyRotError> {
        crate::rotation::Dcm(self.dcm_nb).rotate_ned_to_body(v)
    }

    /// Rotates vector from Body frame to NED frame: v_n = C_b_n * v_b.
    pub fn rotate_body_to_ned(&self, v: [f32; 3]) -> Result<[f32; 3], BodyRotError> {
        crate::rotation::Dcm(self.dcm_nb).rotate_body_to_ned(v)
    }

    /// Transforms body accelerometer specific force to NED linear acceleration.
    pub fn transform_accel_to_ned(
        &self,
        specific_force_body: [f32; 3],
        gravity_ned_z: f32,
    ) -> Result<[f32; 3], BodyRotError> {
        crate::rotation::Dcm(self.dcm_nb).transform_accel_to_ned(specific_force_body, gravity_ned_z)
    }

    /// Reorthonormalizes DCM via symmetric error correction and cross product.
    pub fn reorthonormalize(&self) -> Result<Self, BodyRotError> {
        crate::rotation::Dcm(self.dcm_nb)
            .reorthonormalize()
            .map(|d| Self { dcm_nb: d.0 })
    }
}
