//! Extended Kalman Filter module for IMU/GNSS navigation.

use crate::math::{Mat, Quaternion};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EkfError {
    InvalidInput,
    SingularInnovation,
    OutlierRejected,
}

pub const STATE_DIM: usize = 8;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavEkf {
    pub att: Quaternion,
    pub p_ned_m: [f32; 3],
    pub v_ned_mps: [f32; 3],
    pub bg_z: f32,
    pub cov: Mat<STATE_DIM, STATE_DIM>,
}

impl NavEkf {
    pub fn new() -> Self {
        let mut cov = Mat::<STATE_DIM, STATE_DIM>::zeros();
        cov.0[0][0] = 100.0;
        cov.0[1][1] = 100.0;
        cov.0[2][2] = 100.0;
        cov.0[3][3] = 25.0;
        cov.0[4][4] = 25.0;
        cov.0[5][5] = 25.0;
        cov.0[6][6] = 0.1;
        cov.0[7][7] = 0.01;
        Self {
            att: Quaternion::IDENTITY,
            p_ned_m: [0.0; 3],
            v_ned_mps: [0.0; 3],
            bg_z: 0.0,
            cov,
        }
    }
}

impl Default for NavEkf {
    fn default() -> Self {
        Self::new()
    }
}

fn check_finite_3(v: [f32; 3]) -> Result<(), EkfError> {
    if v[0].is_finite() && v[1].is_finite() && v[2].is_finite() {
        Ok(())
    } else {
        Err(EkfError::InvalidInput)
    }
}

impl NavEkf {
    pub fn update_kinematics(
        &mut self,
        accel_body_mps2: [f32; 3],
        gyro_body_rads: [f32; 3],
        dt_s: f32,
    ) -> Result<(), EkfError> {
        let corrected_gyro = [
            gyro_body_rads[0],
            gyro_body_rads[1],
            gyro_body_rads[2] - self.bg_z,
        ];
        self.att = self
            .att
            .integrate_gyro(corrected_gyro, dt_s)
            .map_err(|_| EkfError::InvalidInput)?;
        let a_nav = self.att.rotate_vec(accel_body_mps2);
        let a_eff = [a_nav[0], a_nav[1], a_nav[2] + 9.80665];
        let half_dt2 = 0.5 * dt_s * dt_s;
        self.p_ned_m[0] += self.v_ned_mps[0] * dt_s + a_eff[0] * half_dt2;
        self.p_ned_m[1] += self.v_ned_mps[1] * dt_s + a_eff[1] * half_dt2;
        self.p_ned_m[2] += self.v_ned_mps[2] * dt_s + a_eff[2] * half_dt2;
        self.v_ned_mps[0] += a_eff[0] * dt_s;
        self.v_ned_mps[1] += a_eff[1] * dt_s;
        self.v_ned_mps[2] += a_eff[2] * dt_s;
        Ok(())
    }
}

impl NavEkf {
    pub fn propagate_covariance(&mut self, dt_s: f32) {
        let mut f = Mat::<STATE_DIM, STATE_DIM>::identity();
        f.0[0][3] = dt_s;
        f.0[1][4] = dt_s;
        f.0[2][5] = dt_s;
        f.0[6][7] = -dt_s;
        let mut q = Mat::<STATE_DIM, STATE_DIM>::zeros();
        q.0[0][0] = 0.01 * dt_s;
        q.0[1][1] = 0.01 * dt_s;
        q.0[2][2] = 0.01 * dt_s;
        q.0[3][3] = 0.1 * dt_s;
        q.0[4][4] = 0.1 * dt_s;
        q.0[5][5] = 0.1 * dt_s;
        q.0[6][6] = 1e-4 * dt_s;
        q.0[7][7] = 1e-6 * dt_s;
        let fp = f.matmul(&self.cov);
        let ft = f.transpose();
        self.cov = fp.matmul(&ft).add(&q).symmetrize();
    }
}

/// Multiple of machine epsilon, relative to the diagonal, below which a pivot is treated as zero.
const PIVOT_FLOOR_EPS: f32 = 16.0;
/// Symmetry tolerance relative to sqrt(P_ii * P_jj).
const SYMMETRY_REL_TOL: f32 = 1e-4;

#[allow(clippy::needless_range_loop)]
fn cholesky_is_positive(m: &Mat<STATE_DIM, STATE_DIM>) -> bool {
    let mut l = [[0.0f32; STATE_DIM]; STATE_DIM];
    for j in 0..STATE_DIM {
        let mut d = m.0[j][j];
        for k in 0..j {
            d -= l[j][k] * l[j][k];
        }
        // Pivot must exceed a relative floor of the original diagonal entry.
        if !d.is_finite() || d <= PIVOT_FLOOR_EPS * f32::EPSILON * libm::fabsf(m.0[j][j]) {
            return false;
        }
        let ljj = libm::sqrtf(d);
        l[j][j] = ljj;
        for i in (j + 1)..STATE_DIM {
            let mut s = m.0[i][j];
            for k in 0..j {
                s -= l[i][k] * l[j][k];
            }
            l[i][j] = s / ljj;
        }
    }
    true
}

impl NavEkf {
    /// Returns true when the covariance is finite, symmetric and positive-definite.
    pub fn covariance_is_positive_definite(&self) -> bool {
        for i in 0..STATE_DIM {
            for j in 0..STATE_DIM {
                let a = self.cov.0[i][j];
                let b = self.cov.0[j][i];
                let scale = libm::sqrtf(libm::fabsf(self.cov.0[i][i] * self.cov.0[j][j]));
                if !a.is_finite() || libm::fabsf(a - b) > SYMMETRY_REL_TOL * scale {
                    return false;
                }
            }
        }
        cholesky_is_positive(&self.cov)
    }
}

#[cfg(test)]
mod cov_pd_tests {
    use super::*;

    #[test]
    fn default_covariance_is_positive_definite() {
        assert!(NavEkf::new().covariance_is_positive_definite());
    }

    #[test]
    fn indefinite_covariance_is_rejected() {
        let mut ekf = NavEkf::new();
        ekf.cov.0[0][3] = 60.0;
        ekf.cov.0[3][0] = 60.0;
        assert!(!ekf.covariance_is_positive_definite());
    }

    #[test]
    fn asymmetric_or_non_finite_covariance_is_rejected() {
        let mut ekf = NavEkf::new();
        ekf.cov.0[2][5] = 1.0;
        assert!(!ekf.covariance_is_positive_definite());
        ekf.cov.0[5][2] = 1.0;
        assert!(ekf.covariance_is_positive_definite());
        ekf.cov.0[4][4] = f32::NAN;
        assert!(!ekf.covariance_is_positive_definite());
    }
}

#[cfg(test)]
mod cov_pd_long_run {
    use super::*;

    #[test]
    fn joseph_covariance_stays_positive_definite_for_20000_epochs() {
        let mut ekf = NavEkf::new();
        let dt = 0.01;
        for step in 0..20_000u32 {
            let t = step as f32 * dt;
            let gyro = [0.0, 0.0, 0.02 + 0.01 * libm::sinf(0.3 * t)];
            ekf.predict([0.1, 0.0, -9.80665], gyro, dt).unwrap();
            if step % 10 == 0 {
                let _ = ekf.update_gnss_pos([0.0; 3], 0.5, 1e6);
                let _ = ekf.update_gnss_vel([0.0; 3], 0.1, 1e6);
            }
            if step % 5 == 0 {
                let _ = ekf.update_baro_altitude(0.0, 0.2, 1e6);
                let _ = ekf.update_mag_yaw(0.0, 0.01, 1e6);
            }
            assert!(ekf.covariance_is_positive_definite(), "epoch {}", step);
        }
    }
}

impl NavEkf {
    pub fn predict(
        &mut self,
        accel_body_mps2: [f32; 3],
        gyro_body_rads: [f32; 3],
        dt_s: f32,
    ) -> Result<(), EkfError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(EkfError::InvalidInput);
        }
        check_finite_3(accel_body_mps2)?;
        check_finite_3(gyro_body_rads)?;
        self.update_kinematics(accel_body_mps2, gyro_body_rads, dt_s)?;
        self.propagate_covariance(dt_s);
        Ok(())
    }
}

impl NavEkf {
    fn apply_scalar_correction(&mut self, k: [f32; STATE_DIM], y: f32) {
        self.p_ned_m[0] += k[0] * y;
        self.p_ned_m[1] += k[1] * y;
        self.p_ned_m[2] += k[2] * y;
        self.v_ned_mps[0] += k[3] * y;
        self.v_ned_mps[1] += k[4] * y;
        self.v_ned_mps[2] += k[5] * y;
        let dyaw = k[6] * y;
        let half_dyaw = 0.5 * dyaw;
        let d_q = Quaternion::new(libm::cosf(half_dyaw), 0.0, 0.0, libm::sinf(half_dyaw));
        if let Ok(new_att) = d_q.mul(&self.att).normalize() {
            self.att = new_att;
        }
        self.bg_z += k[7] * y;
    }
}

impl NavEkf {
    /// Limits the gyro z-bias magnitude and returns true when clamping happened.
    pub fn clamp_gyro_bias(&mut self, max_abs_rads: f32) -> bool {
        if !max_abs_rads.is_finite() || max_abs_rads <= 0.0 || !self.bg_z.is_finite() {
            return false;
        }
        if self.bg_z > max_abs_rads {
            self.bg_z = max_abs_rads;
            true
        } else if self.bg_z < -max_abs_rads {
            self.bg_z = -max_abs_rads;
            true
        } else {
            false
        }
    }
}

#[cfg(test)]
mod gyro_bias_clamp_tests {
    use super::*;

    #[test]
    fn clamp_limits_bias_in_both_directions() {
        let mut ekf = NavEkf::new();
        ekf.bg_z = 0.01;
        assert!(!ekf.clamp_gyro_bias(0.05));
        assert_eq!(ekf.bg_z, 0.01);
        ekf.bg_z = 0.3;
        assert!(ekf.clamp_gyro_bias(0.05));
        assert_eq!(ekf.bg_z, 0.05);
        ekf.bg_z = -0.3;
        assert!(ekf.clamp_gyro_bias(0.05));
        assert_eq!(ekf.bg_z, -0.05);
    }
}

#[cfg(test)]
mod gyro_bias_clamp_invalid_tests {
    use super::*;

    #[test]
    fn invalid_limit_leaves_state_unchanged() {
        let mut ekf = NavEkf::new();
        ekf.bg_z = 0.3;
        let before = ekf;
        for &limit in &[0.0, -0.05, f32::NAN, f32::INFINITY] {
            assert!(!ekf.clamp_gyro_bias(limit));
            assert_eq!(ekf, before);
        }
    }
}

#[cfg(test)]
mod gyro_bias_clamp_scenario_tests {
    use super::*;

    fn heading_burst(limit: Option<f32>) -> f32 {
        let mut ekf = NavEkf::new();
        for step in 0..300u32 {
            ekf.predict([0.0, 0.0, -9.80665], [0.0; 3], 0.02).unwrap();
            let _ = ekf.update_mag_yaw(0.5 * 0.02 * step as f32, 0.001, 1e6);
            if let Some(max_abs) = limit {
                ekf.clamp_gyro_bias(max_abs);
            }
        }
        ekf.bg_z
    }

    #[test]
    fn clamp_bounds_bias_after_corrupted_heading_burst() {
        assert!(libm::fabsf(heading_burst(None)) > 0.1);
        assert!(libm::fabsf(heading_burst(Some(0.05))) <= 0.05);
    }
}

impl NavEkf {
    #[allow(clippy::needless_range_loop)]
    fn apply_scalar_joseph_cov(&mut self, k: [f32; STATE_DIM], h: [f32; STATE_DIM], r: f32) {
        let mut i_kh = Mat::<STATE_DIM, STATE_DIM>::identity();
        for i in 0..STATE_DIM {
            for j in 0..STATE_DIM {
                i_kh.0[i][j] -= k[i] * h[j];
            }
        }
        let ikh_p = i_kh.matmul(&self.cov);
        let ikh_t = i_kh.transpose();
        let term1 = ikh_p.matmul(&ikh_t);
        let mut term2 = Mat::<STATE_DIM, STATE_DIM>::zeros();
        for i in 0..STATE_DIM {
            for j in 0..STATE_DIM {
                term2.0[i][j] = k[i] * k[j] * r;
            }
        }
        self.cov = term1.add(&term2).symmetrize();
    }
}

impl NavEkf {
    #[allow(clippy::needless_range_loop)]
    fn compute_ph_t_and_s(
        &self,
        h: [f32; STATE_DIM],
        r: f32,
    ) -> Result<([f32; STATE_DIM], f32), EkfError> {
        let mut ph_t = [0.0f32; STATE_DIM];
        for i in 0..STATE_DIM {
            let mut sum = 0.0f32;
            for j in 0..STATE_DIM {
                sum += self.cov.0[i][j] * h[j];
            }
            ph_t[i] = sum;
        }
        let mut s = r;
        for i in 0..STATE_DIM {
            s += h[i] * ph_t[i];
        }
        if !s.is_finite() || s < 1e-12 {
            Err(EkfError::SingularInnovation)
        } else {
            Ok((ph_t, s))
        }
    }
}

fn validate_scalar_update(h: &[f32; STATE_DIM], y: f32, r: f32, g: f32) -> Result<(), EkfError> {
    if !y.is_finite() || !r.is_finite() || r <= 0.0 || !g.is_finite() || g <= 0.0 {
        return Err(EkfError::InvalidInput);
    }
    for v in h {
        if !v.is_finite() {
            return Err(EkfError::InvalidInput);
        }
    }
    Ok(())
}

impl NavEkf {
    pub fn update_joseph_scalar(
        &mut self,
        h: [f32; STATE_DIM],
        y: f32,
        r: f32,
        nis_gate: f32,
    ) -> Result<f32, EkfError> {
        validate_scalar_update(&h, y, r, nis_gate)?;
        let (ph_t, s) = self.compute_ph_t_and_s(h, r)?;
        let nis = (y * y) / s;
        if nis > nis_gate {
            return Err(EkfError::OutlierRejected);
        }
        let inv_s = 1.0 / s;
        let mut k = [0.0f32; STATE_DIM];
        for i in 0..STATE_DIM {
            k[i] = ph_t[i] * inv_s;
        }
        self.apply_scalar_correction(k, y);
        self.apply_scalar_joseph_cov(k, h, r);
        Ok(nis)
    }
}

impl NavEkf {
    pub fn update_gnss_pos(
        &mut self,
        p_obs: [f32; 3],
        r_pos: f32,
        gate: f32,
    ) -> Result<[f32; 3], EkfError> {
        check_finite_3(p_obs)?;
        let mut trial = *self;
        let mut nis = [0.0f32; 3];
        for i in 0..3 {
            let mut h = [0.0f32; STATE_DIM];
            h[i] = 1.0;
            let y = p_obs[i] - trial.p_ned_m[i];
            nis[i] = trial.update_joseph_scalar(h, y, r_pos, gate)?;
        }
        *self = trial;
        Ok(nis)
    }
}

impl NavEkf {
    pub fn update_gnss_vel(
        &mut self,
        v_obs: [f32; 3],
        r_vel: f32,
        gate: f32,
    ) -> Result<[f32; 3], EkfError> {
        check_finite_3(v_obs)?;
        let mut trial = *self;
        let mut nis = [0.0f32; 3];
        for i in 0..3 {
            let mut h = [0.0f32; STATE_DIM];
            h[3 + i] = 1.0;
            let y = v_obs[i] - trial.v_ned_mps[i];
            nis[i] = trial.update_joseph_scalar(h, y, r_vel, gate)?;
        }
        *self = trial;
        Ok(nis)
    }
}

impl NavEkf {
    /// Scalar Joseph update for barometric altitude (Down axis: `p_ned_m[2] = -alt`).
    pub fn update_baro_altitude(
        &mut self,
        alt_m: f32,
        r_baro: f32,
        gate: f32,
    ) -> Result<f32, EkfError> {
        if !alt_m.is_finite() {
            return Err(EkfError::InvalidInput);
        }
        let mut h = [0.0f32; STATE_DIM];
        h[2] = 1.0;
        let y = -alt_m - self.p_ned_m[2];
        self.update_joseph_scalar(h, y, r_baro, gate)
    }
}

impl NavEkf {
    /// Scalar Joseph update for vertical velocity (vd = -climb_rate).
    pub fn update_vertical_velocity(
        &mut self,
        v_down_mps: f32,
        r_v_down: f32,
        gate: f32,
    ) -> Result<f32, EkfError> {
        if !v_down_mps.is_finite() {
            return Err(EkfError::InvalidInput);
        }
        let mut h = [0.0f32; STATE_DIM];
        h[5] = 1.0;
        let y = v_down_mps - self.v_ned_mps[2];
        self.update_joseph_scalar(h, y, r_v_down, gate)
    }
}

impl NavEkf {
    pub fn update_mag_yaw(&mut self, yaw_obs: f32, r_yaw: f32, gate: f32) -> Result<f32, EkfError> {
        if !yaw_obs.is_finite() {
            return Err(EkfError::InvalidInput);
        }
        let (_, _, current_yaw) = self.att.to_euler_zyx();
        let mut y = yaw_obs - current_yaw;
        while y > core::f32::consts::PI {
            y -= 2.0 * core::f32::consts::PI;
        }
        while y < -core::f32::consts::PI {
            y += 2.0 * core::f32::consts::PI;
        }
        let mut h = [0.0f32; STATE_DIM];
        h[6] = 1.0;
        self.update_joseph_scalar(h, y, r_yaw, gate)
    }

    pub fn update_dual_antenna_heading(
        &mut self,
        yaw_rad: f32,
        pitch_rad: f32,
        variance: f32,
    ) -> Result<f32, EkfError> {
        if !yaw_rad.is_finite()
            || !pitch_rad.is_finite()
            || !variance.is_finite()
            || variance <= 0.0
        {
            return Err(EkfError::InvalidInput);
        }
        let (_, _, current_yaw) = self.att.to_euler_zyx();
        let mut y = yaw_rad - current_yaw;
        while y > core::f32::consts::PI {
            y -= 2.0 * core::f32::consts::PI;
        }
        while y < -core::f32::consts::PI {
            y += 2.0 * core::f32::consts::PI;
        }
        let mut h = [0.0f32; STATE_DIM];
        h[6] = 1.0;
        self.update_joseph_scalar(h, y, variance, 25.0)
    }

    pub fn fuse_heading_with_declination(
        &mut self,
        mag_yaw_rad: f32,
        mag_variance: f32,
        declination_rad: f32,
        dual_ant_yaw_rad: Option<(f32, f32)>,
    ) -> Result<f32, EkfError> {
        if let Some((dual_yaw, dual_var)) = dual_ant_yaw_rad {
            self.update_dual_antenna_heading(dual_yaw, 0.0, dual_var)
        } else {
            let true_mag_yaw = mag_yaw_rad + declination_rad;
            self.update_mag_yaw(true_mag_yaw, mag_variance, 16.0)
        }
    }
}

impl NavEkf {
    /// One-sigma gyro z-bias uncertainty in rad/s.
    pub fn gyro_bias_sigma(&self) -> f32 {
        libm::sqrtf(self.cov.0[7][7])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_predict_and_symmetry_positive_diagonal() {
        let mut ekf = NavEkf::new();
        let dt = 0.01;
        let accel = [0.0, 0.0, -9.80665];
        let gyro = [0.0, 0.0, 0.0];
        for _ in 0..500 {
            ekf.predict(accel, gyro, dt).unwrap();
            let _ = ekf.update_gnss_pos([0.0, 0.0, 0.0], 1.0, 16.0);
            for i in 0..STATE_DIM {
                assert!(ekf.cov.0[i][i] > 0.0);
                for j in 0..STATE_DIM {
                    assert!((ekf.cov.0[i][j] - ekf.cov.0[j][i]).abs() < 1e-4);
                }
            }
        }
    }

    #[test]
    fn test_nis_spike_rejection() {
        let mut ekf = NavEkf::new();
        ekf.predict([0.0, 0.0, -9.80665], [0.0, 0.0, 0.0], 0.1)
            .unwrap();
        let normal = ekf.update_gnss_pos([0.5, -0.2, 0.1], 1.0, 16.0);
        assert!(normal.is_ok());
        let spike = ekf.update_gnss_pos([50.0, 0.0, 0.0], 1.0, 16.0);
        assert_eq!(spike, Err(EkfError::OutlierRejected));
    }

    #[test]
    fn test_baro_outlier_rejection() {
        let mut ekf = NavEkf::new();
        ekf.predict([0.0, 0.0, -9.80665], [0.0, 0.0, 0.0], 0.1)
            .unwrap();
        assert!(ekf.update_baro_altitude(0.2, 0.5, 9.0).is_ok());
        let spike = ekf.update_baro_altitude(100.0, 0.5, 9.0);
        assert_eq!(spike, Err(EkfError::OutlierRejected));
        let bad = ekf.update_baro_altitude(f32::NAN, 0.5, 9.0);
        assert_eq!(bad, Err(EkfError::InvalidInput));
    }

    #[test]
    fn test_vertical_drift_bounded_under_gnss_outage() {
        let mut ekf = NavEkf::new();
        ekf.p_ned_m[2] = -50.0;
        ekf.cov.0[2][2] = 0.25;
        let dt = 0.01;
        for step in 0..1000 {
            ekf.predict([0.0, 0.0, -9.80665], [0.0, 0.0, 0.0], dt)
                .unwrap();
            if step % 10 == 0 {
                let _ = ekf.update_baro_altitude(50.0, 0.09, 16.0);
            }
        }
        let z_err = (-ekf.p_ned_m[2] - 50.0).abs();
        assert!(z_err < 0.5, "Vertical error {} >= 0.5m", z_err);
    }

    #[test]
    fn test_gyro_bias_convergence() {
        let mut ekf = NavEkf::new();
        let true_bias = 0.03f32;
        let dt = 0.02f32;
        let true_yaw = 0.0f32;
        for _ in 0..400 {
            let omega = [0.0, 0.0, true_bias];
            ekf.predict([0.0, 0.0, -9.80665], omega, dt).unwrap();
            let _ = ekf.update_mag_yaw(true_yaw, 0.001, 100.0);
        }
        assert!((ekf.bg_z - true_bias).abs() < 0.015, "bg_z={}", ekf.bg_z);
    }

    #[test]
    fn test_dual_antenna_heading_convergence() {
        let mut ekf = NavEkf::new();
        let target_yaw = core::f32::consts::FRAC_PI_2;
        for _ in 0..50 {
            ekf.predict([0.0, 0.0, -9.80665], [0.0, 0.0, 0.0], 0.02)
                .unwrap();
            let _ = ekf.update_dual_antenna_heading(target_yaw, 0.0, 0.01);
        }
        let (_, _, est_yaw) = ekf.att.to_euler_zyx();
        assert!((est_yaw - target_yaw).abs() < 0.05);
    }

    #[test]
    fn test_magnetic_anomaly_immunity() {
        let mut ekf = NavEkf::new();
        let true_yaw = 0.5f32;
        let corrupted_mag_yaw = 2.5f32;
        for _ in 0..50 {
            ekf.predict([0.0, 0.0, -9.80665], [0.0, 0.0, 0.0], 0.02)
                .unwrap();
            let _ = ekf.fuse_heading_with_declination(
                corrupted_mag_yaw,
                1.0,
                0.0,
                Some((true_yaw, 0.005)),
            );
        }
        let (_, _, est_yaw) = ekf.att.to_euler_zyx();
        assert!((est_yaw - true_yaw).abs() < 0.05);
    }

    #[test]
    fn test_gnss_update_atomicity() {
        let mut ekf = NavEkf::new();
        let before_pos = ekf;
        assert_eq!(
            ekf.update_gnss_pos([0.5, 5000.0, 0.0], 1.0, 16.0),
            Err(EkfError::OutlierRejected)
        );
        assert_eq!(ekf, before_pos);

        let before_vel = ekf;
        assert_eq!(
            ekf.update_gnss_vel([0.5, 5000.0, 0.0], 1.0, 16.0),
            Err(EkfError::OutlierRejected)
        );
        assert_eq!(ekf, before_vel);
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StationaryDetector {
    pub window_size: usize,
    pub count: usize,
    pub var_accel_thresh: f32,
    pub norm_gyro_thresh: f32,
    sum_accel_mag: f32,
    sum_accel_mag_sq: f32,
    max_gyro_norm: f32,
}

impl StationaryDetector {
    pub const fn new(window_size: usize, var_accel_thresh: f32, norm_gyro_thresh: f32) -> Self {
        Self {
            window_size,
            count: 0,
            var_accel_thresh,
            norm_gyro_thresh,
            sum_accel_mag: 0.0,
            sum_accel_mag_sq: 0.0,
            max_gyro_norm: 0.0,
        }
    }
}

impl StationaryDetector {
    pub fn reset(&mut self) {
        self.count = 0;
        self.sum_accel_mag = 0.0;
        self.sum_accel_mag_sq = 0.0;
        self.max_gyro_norm = 0.0;
    }
}

impl Default for StationaryDetector {
    fn default() -> Self {
        Self::new(10, 0.05, 0.03)
    }
}

impl StationaryDetector {
    pub fn update(&mut self, accel_body: [f32; 3], gyro_body: [f32; 3]) -> bool {
        let a_mag = libm::sqrtf(
            accel_body[0] * accel_body[0]
                + accel_body[1] * accel_body[1]
                + accel_body[2] * accel_body[2],
        );
        let g_norm = libm::sqrtf(
            gyro_body[0] * gyro_body[0] + gyro_body[1] * gyro_body[1] + gyro_body[2] * gyro_body[2],
        );
        self.sum_accel_mag += a_mag;
        self.sum_accel_mag_sq += a_mag * a_mag;
        if g_norm > self.max_gyro_norm {
            self.max_gyro_norm = g_norm;
        }
        self.count += 1;
        if self.count >= self.window_size {
            let mean = self.sum_accel_mag / (self.count as f32);
            let var = (self.sum_accel_mag_sq / (self.count as f32)) - (mean * mean);
            let stat = var < self.var_accel_thresh && self.max_gyro_norm < self.norm_gyro_thresh;
            self.reset();
            stat
        } else {
            false
        }
    }
}

impl NavEkf {
    pub fn update_zupt(&mut self, r_zupt: f32, gate: f32) -> Result<[f32; 3], EkfError> {
        let mut trial = *self;
        let mut nis = [0.0f32; 3];
        for i in 0..3 {
            let mut h = [0.0f32; STATE_DIM];
            h[3 + i] = 1.0;
            let y = 0.0 - trial.v_ned_mps[i];
            nis[i] = trial.update_joseph_scalar(h, y, r_zupt, gate)?;
        }
        *self = trial;
        Ok(nis)
    }
}

#[cfg(test)]
mod zupt_tests {
    use super::*;

    #[test]
    fn test_stationary_detector_detects_static_motion() {
        let mut detector = StationaryDetector::new(5, 0.05, 0.03);
        let accel = [0.0, 0.0, -9.80665];
        let gyro = [0.001, -0.001, 0.0];
        let mut detected = false;
        for _ in 0..5 {
            detected = detector.update(accel, gyro);
        }
        assert!(detected);
    }
}

#[cfg(test)]
mod zupt_tests_dynamic {
    use super::*;

    #[test]
    fn test_stationary_detector_rejects_high_motion() {
        let mut detector = StationaryDetector::new(5, 0.05, 0.03);
        let accel = [2.5, 0.0, -9.8];
        let gyro = [0.2, 0.0, 0.0];
        let mut detected = false;
        for _ in 0..5 {
            detected = detector.update(accel, gyro);
        }
        assert!(!detected);
    }
}

#[cfg(test)]
mod zupt_tests_clamp {
    use super::*;

    #[test]
    fn test_update_zupt_clamps_velocity() {
        let mut ekf = NavEkf::new();
        ekf.v_ned_mps = [0.5, -0.4, 0.1];
        let res = ekf.update_zupt(0.001, 25.0);
        assert!(res.is_ok());
        assert!(ekf.v_ned_mps[0].abs() < 0.01);
        assert!(ekf.v_ned_mps[1].abs() < 0.01);
        assert!(ekf.cov.0[3][3] < 1.0);
    }
}

#[cfg(test)]
mod zupt_tests_drift {
    use super::*;

    #[test]
    fn test_zero_velocity_drift_5000_epochs() {
        let mut ekf = NavEkf::new();
        let mut detector = StationaryDetector::new(10, 0.05, 0.03);
        let dt = 0.01;
        let accel = [0.0, 0.0, -9.80665];
        let gyro = [0.0, 0.0, 0.0];
        for _ in 0..5000 {
            ekf.predict(accel, gyro, dt).unwrap();
            if detector.update(accel, gyro) {
                ekf.update_zupt(0.0001, 25.0).unwrap();
            }
        }
        assert!(ekf.v_ned_mps[0].abs() < 1e-4);
        assert!(ekf.v_ned_mps[1].abs() < 1e-4);
        assert!(ekf.v_ned_mps[2].abs() < 1e-4);
    }
}

/// Opt-in per-axis scalar Kalman filter for the body-frame accelerometer bias.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AccelBiasEstimator {
    pub bias_mps2: [f32; 3],
    pub var_mps2_sq: [f32; 3],
    pub r_mps2_sq: f32,
}

impl AccelBiasEstimator {
    pub const fn new(prior_var: f32, r_mps2_sq: f32) -> Self {
        Self {
            bias_mps2: [0.0; 3],
            var_mps2_sq: [prior_var; 3],
            r_mps2_sq,
        }
    }
}

impl AccelBiasEstimator {
    /// Removes the estimated bias from a raw body-frame accelerometer sample.
    pub fn corrected(&self, accel_body: [f32; 3]) -> [f32; 3] {
        [
            accel_body[0] - self.bias_mps2[0],
            accel_body[1] - self.bias_mps2[1],
            accel_body[2] - self.bias_mps2[2],
        ]
    }
}

impl AccelBiasEstimator {
    /// Fuses one stationary sample against the expected specific force R^T * [0, 0, -g].
    #[allow(clippy::needless_range_loop)]
    pub fn update(&mut self, att: &Quaternion, accel_body: [f32; 3]) -> Result<(), EkfError> {
        check_finite_3(accel_body)?;
        if !self.r_mps2_sq.is_finite() || self.r_mps2_sq <= 0.0 {
            return Err(EkfError::InvalidInput);
        }
        let conj = Quaternion::new(att.w, -att.x, -att.y, -att.z);
        let expected = conj.rotate_vec([0.0, 0.0, -9.80665]);
        check_finite_3(expected)?;
        for i in 0..3 {
            let k = self.var_mps2_sq[i] / (self.var_mps2_sq[i] + self.r_mps2_sq);
            self.bias_mps2[i] += k * (accel_body[i] - expected[i] - self.bias_mps2[i]);
            self.var_mps2_sq[i] *= 1.0 - k;
        }
        Ok(())
    }
}

#[cfg(test)]
mod accel_bias_tests {
    use super::*;

    const BIAS: [f32; 3] = [0.12, -0.08, 0.2];

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn converges_to_known_bias_under_noise() {
        let mut est = AccelBiasEstimator::new(1.0, 0.01);
        let mut seed = 0x2468_ace1u32;
        for _ in 0..2000 {
            seed = seed.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let n = ((seed >> 8) as f32 / 16_777_216.0 - 0.5) * 0.2;
            let accel = [BIAS[0] + n, BIAS[1] - n, -9.80665 + BIAS[2] + 0.5 * n];
            est.update(&Quaternion::IDENTITY, accel).unwrap();
        }
        for i in 0..3 {
            assert!((est.bias_mps2[i] - BIAS[i]).abs() < 0.02, "axis {}", i);
            assert!(est.var_mps2_sq[i] < 1e-4, "axis {}", i);
        }
    }
}

#[cfg(test)]
mod accel_bias_tilt_tests {
    use super::*;

    #[test]
    #[allow(clippy::needless_range_loop)]
    fn tilted_attitude_uses_rotated_gravity() {
        let roll = 0.3f32;
        let att = Quaternion::new(libm::cosf(0.5 * roll), libm::sinf(0.5 * roll), 0.0, 0.0);
        let g = 9.80665f32;
        let g_body = [0.0, -g * libm::sinf(roll), -g * libm::cosf(roll)];
        let raw = [g_body[0] + 0.1, g_body[1] - 0.05, g_body[2] + 0.07];
        let mut est = AccelBiasEstimator::new(1.0, 0.01);
        for _ in 0..200 {
            est.update(&att, raw).unwrap();
        }
        let c = est.corrected(raw);
        for i in 0..3 {
            assert!((c[i] - g_body[i]).abs() < 0.01, "axis {}", i);
        }
    }
}

#[cfg(test)]
mod accel_bias_reject_tests {
    use super::*;

    #[test]
    fn invalid_input_leaves_estimator_unchanged() {
        let mut est = AccelBiasEstimator::new(1.0, 0.01);
        let id = Quaternion::IDENTITY;
        est.update(&id, [0.1, 0.0, -9.7]).unwrap();
        let before = est;
        let bad = est.update(&id, [f32::NAN, 0.0, -9.8]);
        assert_eq!(bad, Err(EkfError::InvalidInput));
        let nan_att = Quaternion::new(f32::NAN, 0.0, 0.0, 0.0);
        let bad = est.update(&nan_att, [0.0, 0.0, -9.8]);
        assert_eq!(bad, Err(EkfError::InvalidInput));
        est.r_mps2_sq = 0.0;
        let bad = est.update(&id, [0.0, 0.0, -9.8]);
        assert_eq!(bad, Err(EkfError::InvalidInput));
        est.r_mps2_sq = before.r_mps2_sq;
        assert_eq!(est, before);
    }
}
