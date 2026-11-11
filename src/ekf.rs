//! Extended Kalman Filter module for IMU/GNSS navigation.

use crate::math::{Mat, Quaternion};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EkfError {
    InvalidInput,
    SingularInnovation,
    OutlierRejected,
}

pub const STATE_DIM: usize = 7;

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
        cov.0[6][6] = 0.01;
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
        self.att = self.att.integrate_gyro(corrected_gyro, dt_s).map_err(|_| EkfError::InvalidInput)?;
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
        let mut q = Mat::<STATE_DIM, STATE_DIM>::zeros();
        q.0[0][0] = 0.01 * dt_s;
        q.0[1][1] = 0.01 * dt_s;
        q.0[2][2] = 0.01 * dt_s;
        q.0[3][3] = 0.1 * dt_s;
        q.0[4][4] = 0.1 * dt_s;
        q.0[5][5] = 0.1 * dt_s;
        q.0[6][6] = 1e-6 * dt_s;
        let fp = f.matmul(&self.cov);
        let ft = f.transpose();
        self.cov = fp.matmul(&ft).add(&q).symmetrize();
    }
}
