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
