//! Attitude covariance propagation under angular rate noise.

use super::AttitudeError;
use crate::math::{Mat, Quaternion};

/// Propagate 3x3 orientation error covariance under rate noise variance and timestep.
pub fn propagate_attitude_cov(
    cov: &Mat<3, 3>,
    q: &Quaternion,
    gyro_var: f32,
    dt: f32,
) -> Result<Mat<3, 3>, AttitudeError> {
    if dt <= 0.0 || gyro_var < 0.0 || !dt.is_finite() || !gyro_var.is_finite() {
        return Err(AttitudeError::InvalidInput);
    }
    if !cov.0.iter().flatten().all(|v| v.is_finite()) {
        return Err(AttitudeError::NonFinite);
    }
    let _ = q.normalize().map_err(|_| AttitudeError::InvalidInput)?;

    let mut out = *cov;
    let noise = gyro_var * dt;
    out.0[0][0] += noise;
    out.0[1][1] += noise;
    out.0[2][2] += noise;
    Ok(out.symmetrize())
}
