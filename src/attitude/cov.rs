//! Attitude covariance propagation under isotropic angular rate noise.

use super::AttitudeError;
use crate::math::Mat;

/// Propagate 3x3 orientation error covariance under isotropic rate noise.
///
/// Adds isotropic process-noise inflation `gyro_var * dt` on each diagonal entry.
/// `gyro_var` is the angular rate noise variance in (rad/s)^2, and `dt` is the
/// timestep in seconds. The orientation error covariance is assumed to be in the
/// body frame under an identity error state transition.
pub fn propagate_attitude_cov(
    cov: &Mat<3, 3>,
    gyro_var: f32,
    dt: f32,
) -> Result<Mat<3, 3>, AttitudeError> {
    if dt <= 0.0 || gyro_var < 0.0 || !dt.is_finite() || !gyro_var.is_finite() {
        return Err(AttitudeError::InvalidInput);
    }
    if !cov.0.iter().flatten().all(|v| v.is_finite()) {
        return Err(AttitudeError::NonFinite);
    }

    let mut out = *cov;
    let noise = gyro_var * dt;
    out.0[0][0] += noise;
    out.0[1][1] += noise;
    out.0[2][2] += noise;
    Ok(out.symmetrize())
}
