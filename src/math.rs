//! Math module: Matrix algebra, Quaternion, and tilt-compensated heading.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MathError {
    SingularMatrix,
    InvalidNorm,
    GimbalLock,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mat<const R: usize, const C: usize>(pub [[f32; C]; R]);

impl<const R: usize, const C: usize> Mat<R, C> {
    pub const fn zeros() -> Self {
        Self([[0.0; C]; R])
    }

    pub fn identity() -> Self {
        let mut out = [[0.0; C]; R];
        let n = if R < C { R } else { C };
        let mut i = 0;
        while i < n {
            out[i][i] = 1.0;
            i += 1;
        }
        Self(out)
    }
}

impl<const R: usize, const C: usize> Mat<R, C> {
    #[allow(clippy::needless_range_loop)]
    pub fn add(&self, other: &Self) -> Self {
        let mut out = [[0.0; C]; R];
        for r in 0..R {
            for c in 0..C {
                out[r][c] = self.0[r][c] + other.0[r][c];
            }
        }
        Self(out)
    }

    #[allow(clippy::needless_range_loop)]
    pub fn sub(&self, other: &Self) -> Self {
        let mut out = [[0.0; C]; R];
        for r in 0..R {
            for c in 0..C {
                out[r][c] = self.0[r][c] - other.0[r][c];
            }
        }
        Self(out)
    }
}

impl<const R: usize, const C: usize> Mat<R, C> {
    #[allow(clippy::needless_range_loop)]
    pub fn matmul<const K: usize>(&self, other: &Mat<C, K>) -> Mat<R, K> {
        let mut out = [[0.0; K]; R];
        for r in 0..R {
            for k in 0..K {
                let mut sum = 0.0f32;
                for c in 0..C {
                    sum += self.0[r][c] * other.0[c][k];
                }
                out[r][k] = sum;
            }
        }
        Mat(out)
    }
}

impl<const R: usize, const C: usize> Mat<R, C> {
    #[allow(clippy::needless_range_loop)]
    pub fn transpose(&self) -> Mat<C, R> {
        let mut out = [[0.0; R]; C];
        for r in 0..R {
            for c in 0..C {
                out[c][r] = self.0[r][c];
            }
        }
        Mat(out)
    }
}

impl<const N: usize> Mat<N, N> {
    #[allow(clippy::needless_range_loop)]
    pub fn symmetrize(&self) -> Self {
        let mut out = [[0.0; N]; N];
        for r in 0..N {
            for c in 0..N {
                out[r][c] = 0.5 * (self.0[r][c] + self.0[c][r]);
            }
        }
        Mat(out)
    }
}

impl Mat<1, 1> {
    pub fn inv_1x1(&self) -> Option<Self> {
        let v = self.0[0][0];
        let abs_v = if v < 0.0 { -v } else { v };
        if abs_v < 1e-12 {
            None
        } else {
            Some(Mat([[1.0 / v]]))
        }
    }
}

impl Mat<2, 2> {
    pub fn inv_2x2(&self) -> Option<Self> {
        let det = self.0[0][0] * self.0[1][1] - self.0[0][1] * self.0[1][0];
        let abs_det = if det < 0.0 { -det } else { det };
        if abs_det < 1e-12 {
            return None;
        }
        let inv_det = 1.0 / det;
        Some(Mat([
            [self.0[1][1] * inv_det, -self.0[0][1] * inv_det],
            [-self.0[1][0] * inv_det, self.0[0][0] * inv_det],
        ]))
    }
}

impl Mat<3, 3> {
    pub fn inv_3x3(&self) -> Option<Self> {
        let m = &self.0;
        let c00 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
        let c01 = m[1][2] * m[2][0] - m[1][0] * m[2][2];
        let c02 = m[1][0] * m[2][1] - m[1][1] * m[2][0];
        let det = m[0][0] * c00 + m[0][1] * c01 + m[0][2] * c02;
        let abs_det = if det < 0.0 { -det } else { det };
        if abs_det < 1e-12 {
            return None;
        }
        let inv_det = 1.0 / det;
        let c10 = m[0][2] * m[2][1] - m[0][1] * m[2][2];
        let c11 = m[0][0] * m[2][2] - m[0][2] * m[2][0];
        let c12 = m[0][1] * m[2][0] - m[0][0] * m[2][1];
        let c20 = m[0][1] * m[1][2] - m[0][2] * m[1][1];
        let c21 = m[0][2] * m[1][0] - m[0][0] * m[1][2];
        let c22 = m[0][0] * m[1][1] - m[0][1] * m[1][0];
        Some(Mat([
            [c00 * inv_det, c10 * inv_det, c20 * inv_det],
            [c01 * inv_det, c11 * inv_det, c21 * inv_det],
            [c02 * inv_det, c12 * inv_det, c22 * inv_det],
        ]))
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quaternion {
    pub w: f32,
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Quaternion {
    pub const IDENTITY: Self = Self {
        w: 1.0,
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub const fn new(w: f32, x: f32, y: f32, z: f32) -> Self {
        Self { w, x, y, z }
    }
}

impl Quaternion {
    pub fn mul(&self, rhs: &Self) -> Self {
        Self {
            w: self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
            x: self.w * rhs.x + self.x * rhs.w + self.y * rhs.z - self.z * rhs.y,
            y: self.w * rhs.y - self.x * rhs.z + self.y * rhs.w + self.z * rhs.x,
            z: self.w * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.w,
        }
    }
}

impl Quaternion {
    pub fn norm(&self) -> f32 {
        libm::sqrtf(self.w * self.w + self.x * self.x + self.y * self.y + self.z * self.z)
    }

    pub fn normalize(&self) -> Result<Self, MathError> {
        let n = self.norm();
        if n < 1e-9 {
            return Err(MathError::InvalidNorm);
        }
        let inv_n = 1.0 / n;
        Ok(Self {
            w: self.w * inv_n,
            x: self.x * inv_n,
            y: self.y * inv_n,
            z: self.z * inv_n,
        })
    }
}

impl Quaternion {
    pub fn integrate_gyro(&self, omega_rad_s: [f32; 3], dt_s: f32) -> Result<Self, MathError> {
        let wx = omega_rad_s[0];
        let wy = omega_rad_s[1];
        let wz = omega_rad_s[2];
        let theta = libm::sqrtf(wx * wx + wy * wy + wz * wz) * dt_s;
        let delta = if theta < 1e-6 {
            let half_dt = 0.5 * dt_s;
            Quaternion::new(1.0, wx * half_dt, wy * half_dt, wz * half_dt)
        } else {
            let half_theta = 0.5 * theta;
            let sin_term = libm::sinf(half_theta) / (theta / dt_s);
            Quaternion::new(libm::cosf(half_theta), wx * sin_term, wy * sin_term, wz * sin_term)
        };
        self.mul(&delta).normalize()
    }
}

impl Quaternion {
    pub fn rotate_vec(&self, v: [f32; 3]) -> [f32; 3] {
        let p = Quaternion::new(0.0, v[0], v[1], v[2]);
        let conj = Quaternion::new(self.w, -self.x, -self.y, -self.z);
        let res = self.mul(&p).mul(&conj);
        [res.x, res.y, res.z]
    }
}

impl Quaternion {
    pub fn to_euler_zyx(&self) -> (f32, f32, f32) {
        let sinr_cosp = 2.0 * (self.w * self.x + self.y * self.z);
        let cosr_cosp = 1.0 - 2.0 * (self.x * self.x + self.y * self.y);
        let roll = libm::atan2f(sinr_cosp, cosr_cosp);

        let sinp = 2.0 * (self.w * self.y - self.z * self.x);
        let pitch = if sinp >= 1.0 {
            core::f32::consts::FRAC_PI_2
        } else if sinp <= -1.0 {
            -core::f32::consts::FRAC_PI_2
        } else {
            libm::asinf(sinp)
        };

        let siny_cosp = 2.0 * (self.w * self.z + self.x * self.y);
        let cosy_cosp = 1.0 - 2.0 * (self.y * self.y + self.z * self.z);
        let yaw = libm::atan2f(siny_cosp, cosy_cosp);

        (roll, pitch, yaw)
    }
}

pub fn calibrate_magnetometer(
    b_raw: [f32; 3],
    hard_iron: [f32; 3],
    soft_iron: Mat<3, 3>,
) -> [f32; 3] {
    let unb = [
        b_raw[0] - hard_iron[0],
        b_raw[1] - hard_iron[1],
        b_raw[2] - hard_iron[2],
    ];
    let diff_mat = Mat([[unb[0]], [unb[1]], [unb[2]]]);
    let cal = soft_iron.matmul(&diff_mat);
    [cal.0[0][0], cal.0[1][0], cal.0[2][0]]
}

pub fn tilt_compensated_yaw(
    b_raw: [f32; 3],
    hard_iron: [f32; 3],
    soft_iron: Mat<3, 3>,
    roll: f32,
    pitch: f32,
) -> Result<f32, MathError> {
    let cp = libm::cosf(pitch);
    if (if cp < 0.0 { -cp } else { cp }) < 1e-4 {
        return Err(MathError::GimbalLock);
    }
    let b = calibrate_magnetometer(b_raw, hard_iron, soft_iron);
    let (sr, cr) = (libm::sinf(roll), libm::cosf(roll));
    let sp = libm::sinf(pitch);
    let b_xh = b[0] * cp + b[1] * sp * sr + b[2] * sp * cr;
    let b_yh = b[1] * cr - b[2] * sr;
    if libm::sqrtf(b_xh * b_xh + b_yh * b_yh) < 1e-6 {
        return Err(MathError::InvalidNorm);
    }
    Ok(libm::atan2f(-b_yh, b_xh))
}
