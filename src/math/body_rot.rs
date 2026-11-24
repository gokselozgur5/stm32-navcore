//! Backward compatibility aliases and wrappers for [`crate::math::rotation::Dcm`].
//!
//! Deprecated: use [`crate::math::rotation::Dcm`] and [`crate::math::rotation::RotationError`].

pub use crate::math::rotation::dcm::DCM_TOLERANCE as BODY_ROT_TOLERANCE;
pub use crate::math::rotation::RotationError as BodyRotError;

/// Compatibility wrapper around [`crate::math::rotation::Dcm`] with field `dcm_nb`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyRot {
    pub dcm_nb: crate::math::Mat<3, 3>,
}

impl BodyRot {
    /// Creates an identity rotation where NED and Body frames align.
    pub fn identity() -> Self {
        Self {
            dcm_nb: crate::math::Mat::identity(),
        }
    }

    /// Returns a reference to the NED-to-Body DCM (C_n_b).
    pub fn dcm_ned_to_body(&self) -> &crate::math::Mat<3, 3> {
        &self.dcm_nb
    }

    /// Returns the transposed Body-to-NED DCM (C_b_n).
    pub fn dcm_body_to_ned(&self) -> crate::math::Mat<3, 3> {
        self.dcm_nb.transpose()
    }

    /// Computes determinant of the 3x3 rotation matrix.
    pub fn det(&self) -> f32 {
        crate::math::rotation::Dcm(self.dcm_nb).det()
    }

    /// Checks if inner DCM satisfies orthonormality.
    pub fn is_orthonormal(&self) -> bool {
        crate::math::rotation::Dcm(self.dcm_nb).is_orthonormal()
    }

    /// Builds a rotation from an existing NED-to-Body DCM.
    pub fn from_dcm_ned_to_body(m: crate::math::Mat<3, 3>) -> Result<Self, BodyRotError> {
        crate::math::rotation::Dcm::from_dcm_ned_to_body(m).map(|d| Self { dcm_nb: d.0 })
    }

    /// Builds a rotation from an existing Body-to-NED DCM.
    pub fn from_dcm_body_to_ned(m: crate::math::Mat<3, 3>) -> Result<Self, BodyRotError> {
        crate::math::rotation::Dcm::from_dcm_body_to_ned(m).map(|d| Self { dcm_nb: d.0 })
    }

    /// Builds NED-to-Body rotation from Euler angles (roll, pitch, yaw).
    pub fn from_euler(roll: f32, pitch: f32, yaw: f32) -> Result<Self, BodyRotError> {
        crate::math::rotation::Dcm::from_euler(roll, pitch, yaw).map(|d| Self { dcm_nb: d.0 })
    }

    /// Extracts Euler angles (roll, pitch, yaw) in radians.
    pub fn to_euler(&self) -> Result<(f32, f32, f32), BodyRotError> {
        crate::math::rotation::Dcm(self.dcm_nb).to_euler()
    }

    /// Rotates vector from NED frame to Body frame: v_b = C_n_b * v_n.
    pub fn rotate_ned_to_body(&self, v: [f32; 3]) -> Result<[f32; 3], BodyRotError> {
        crate::math::rotation::Dcm(self.dcm_nb).rotate_ned_to_body(v)
    }

    /// Rotates vector from Body frame to NED frame: v_n = C_b_n * v_b.
    pub fn rotate_body_to_ned(&self, v: [f32; 3]) -> Result<[f32; 3], BodyRotError> {
        crate::math::rotation::Dcm(self.dcm_nb).rotate_body_to_ned(v)
    }

    /// Transforms body accelerometer specific force to NED linear acceleration.
    pub fn transform_accel_to_ned(
        &self,
        specific_force_body: [f32; 3],
        gravity_ned_z: f32,
    ) -> Result<[f32; 3], BodyRotError> {
        crate::math::rotation::Dcm(self.dcm_nb)
            .transform_accel_to_ned(specific_force_body, gravity_ned_z)
    }

    /// Reorthonormalizes DCM via symmetric error correction and cross product.
    pub fn reorthonormalize(&self) -> Result<Self, BodyRotError> {
        crate::math::rotation::Dcm(self.dcm_nb)
            .reorthonormalize()
            .map(|d| Self { dcm_nb: d.0 })
    }
}
