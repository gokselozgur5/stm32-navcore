//! Magnetic declination polynomial approximation for embedded navigation.

use libm::{atan2f, cosf, sinf, sqrtf};

#[rustfmt::skip]
const WMM2020_COEFFS: [[f32; 13]; 13] = [
    [0.00, -29404.50, -3750.00, 3409.75, 3951.06, -1845.90, 951.43, 2161.09, 1186.45, 474.80, -342.81, 1033.35, -1320.39],
    [4652.90, -1450.70, 5164.98, -7290.29, 4479.21, 3691.49, 1240.04, -2724.07, 656.91, 1044.71, -1508.37, -652.94, -89.70],
    [-5181.60, -636.36, 1452.15, 2393.89, 337.31, 1443.28, 1090.93, -240.37, -981.44, 315.09, -21.07, -1022.62, 397.56],
    [-251.69, 468.24, -429.20, 415.60, -647.16, -662.16, -1210.48, 1157.03, -16.57, -116.18, 280.98, 787.12, 843.99],
    [1560.58, -619.84, 417.91, -258.90, 35.42, -335.44, -197.54, 195.11, -564.13, -62.01, -105.18, -215.56, -584.30],
    [484.95, 1601.60, -570.86, 71.44, 69.52, 9.61, 31.41, 39.52, 226.91, -448.09, 44.35, 47.53, 233.81],
    [-361.05, 373.61, 525.04, -351.42, 20.94, 45.74, -43.46, -17.44, 94.05, 19.14, -37.19, -65.88, 62.49],
    [-1823.14, -486.54, 47.10, 290.20, -13.58, -65.87, -1.23, 6.34, -41.36, 67.05, 38.08, -4.96, 58.53],
    [563.06, -858.06, 530.17, -315.49, 220.98, 24.71, -17.30, 1.75, -0.19, -24.03, 11.46, 31.86, -11.71],
    [-2968.50, 1206.02, 813.23, -287.52, -208.88, 135.70, 3.01, -3.88, 5.91, -7.25, -6.37, -5.29, -12.77],
    [827.17, -42.14, 578.48, 560.98, -635.67, -4.13, -84.18, -27.82, -0.27, -5.22, -2.32, 0.54, 0.94],
    [-0.00, 1063.52, -163.98, -95.81, 95.05, -18.82, -84.33, -36.42, -26.44, -5.44, -1.51, 1.80, -3.06],
    [-1076.43, 397.56, 843.99, -876.45, 33.40, 145.81, -11.71, 35.12, 5.11, -8.49, -0.00, 0.28, -0.17],
];

#[inline]
fn compute_wmm_harmonics(
    n: usize,
    st: f32,
    ct: f32,
    p: &mut [f32; 169],
    dp: &mut [[f32; 13]; 13],
    pp: &mut [f32; 13],
) {
    for m in 0..=n {
        if n == m {
            p[n + m * 13] = st * p[n - 1 + (m - 1) * 13];
            dp[m][n] = st * dp[m - 1][n - 1] + ct * p[n - 1 + (m - 1) * 13];
        } else if n == 1 && m == 0 {
            p[n + m * 13] = ct * p[n - 1 + m * 13];
            dp[m][n] = ct * dp[m][n - 1] - st * p[n - 1 + m * 13];
        } else {
            let k = (((n - 1) * (n - 1) - m * m) as f32) / (((2 * n - 1) * (2 * n - 3)) as f32);
            p[n + m * 13] = ct * p[n - 1 + m * 13] - k * p[n - 2 + m * 13];
            dp[m][n] = ct * dp[m][n - 1] - st * p[n - 1 + m * 13] - k * dp[m][n - 2];
        }
        if st == 0.0 && m == 1 {
            let k1 = (((n - 1) * (n - 1) - 1) as f32) / (((2 * n - 1) * (2 * n - 3)) as f32);
            pp[n] = if n == 1 {
                pp[n - 1]
            } else {
                ct * pp[n - 1] - k1 * pp[n - 2]
            };
        }
    }
}

#[inline]
fn accumulate_wmm_order(
    n: usize,
    m: usize,
    ar: f32,
    p_val: f32,
    dp_val: f32,
    cp_m: f32,
    sp_m: f32,
    bt: &mut f32,
    bp: &mut f32,
    br: &mut f32,
) {
    let par = ar * p_val;
    let (t1, t2) = if m == 0 {
        (WMM2020_COEFFS[m][n] * cp_m, WMM2020_COEFFS[m][n] * sp_m)
    } else {
        (
            WMM2020_COEFFS[m][n] * cp_m + WMM2020_COEFFS[n][m - 1] * sp_m,
            WMM2020_COEFFS[m][n] * sp_m - WMM2020_COEFFS[n][m - 1] * cp_m,
        )
    };
    *bt -= ar * t1 * dp_val;
    *bp += (m as f32) * t2 * par;
    *br += ((n + 1) as f32) * t1 * par;
}

#[inline]
fn compute_wmm_geom(
    lat_deg: f32,
    lon_deg: f32,
) -> (f32, f32, f32, f32, f32, f32, [f32; 13], [f32; 13]) {
    let dtr = core::f32::consts::PI / 180.0;
    let (srlon, crlon) = (sinf(lon_deg * dtr), cosf(lon_deg * dtr));
    let (srlat, crlat) = (sinf(lat_deg * dtr), cosf(lat_deg * dtr));
    let mut sp = [0.0f32; 13];
    let mut cp = [0.0f32; 13];
    cp[0] = 1.0;
    sp[1] = srlon;
    cp[1] = crlon;
    for m in 2..=12 {
        sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1];
        cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1];
    }
    let (a, b, re) = (6378.137f32, 6356.7523f32, 6371.2f32);
    let (a2, b2) = (a * a, b * b);
    let (c2, a4, c4) = (a2 - b2, a2 * a2, a2 * a2 - b2 * b2);
    let (srlat2, crlat2) = (srlat * srlat, crlat * crlat);
    let q = sqrtf(a2 - c2 * srlat2);
    let q2 = (a2 / b2) * (a2 / b2);
    let ct = srlat / sqrtf(q2 * crlat2 + srlat2);
    let st = sqrtf((1.0 - ct * ct).max(0.0));
    let r = sqrtf((a4 - c4 * srlat2) / (q * q));
    let d = sqrtf(a2 * crlat2 + b2 * srlat2);
    (
        ct,
        st,
        d / r,
        c2 * crlat * srlat / (r * d),
        re / r,
        dtr,
        sp,
        cp,
    )
}

#[inline]
fn eval_wmm_field(lat_deg: f32, lon_deg: f32) -> f32 {
    let (ct, st, ca, sa, aor, dtr, sp, cp) = compute_wmm_geom(lat_deg, lon_deg);
    let mut ar = aor * aor;
    let (mut br, mut bt, mut bp, mut bpp) = (0.0f32, 0.0f32, 0.0f32, 0.0f32);
    let mut p = [0.0f32; 169];
    let mut dp = [[0.0f32; 13]; 13];
    p[0] = 1.0;
    let mut pp = [0.0f32; 13];
    pp[0] = 1.0;
    for n in 1..=12 {
        ar *= aor;
        compute_wmm_harmonics(n, st, ct, &mut p, &mut dp, &mut pp);
        for m in 0..=n {
            accumulate_wmm_order(
                n,
                m,
                ar,
                p[n + m * 13],
                dp[m][n],
                cp[m],
                sp[m],
                &mut bt,
                &mut bp,
                &mut br,
            );
            if st == 0.0 && m == 1 {
                let t2 = WMM2020_COEFFS[m][n] * sp[m] - WMM2020_COEFFS[n][m - 1] * cp[m];
                bpp += (m as f32) * t2 * ar * pp[n];
            }
        }
    }
    if st == 0.0 {
        bp = bpp;
    } else {
        bp /= st;
    }
    atan2f(bp, -bt * ca - br * sa) / dtr
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeclinationError {
    NonFinite,
    InvalidLatitude,
    InvalidLongitude,
    InvalidScale,
}

/// Normalize an angle in degrees into [0.0, 360.0).
#[inline]
pub fn normalize_heading_deg(deg: f32) -> f32 {
    let rem = deg % 360.0;
    let res = if rem < 0.0 { rem + 360.0 } else { rem };
    if res >= 360.0 { 0.0 } else { res }
}

/// Normalize an angle in radians into [0.0, 2*PI).
#[inline]
pub fn normalize_heading_rad(rad: f32) -> f32 {
    let two_pi = 2.0 * core::f32::consts::PI;
    let rem = rad % two_pi;
    let res = if rem < 0.0 { rem + two_pi } else { rem };
    if res >= two_pi { 0.0 } else { res }
}

/// Compute true heading from magnetic heading and declination (in degrees).
pub fn magnetic_to_true_heading_deg(mag_deg: f32, dec_deg: f32) -> Result<f32, DeclinationError> {
    if !mag_deg.is_finite() || !dec_deg.is_finite() {
        return Err(DeclinationError::NonFinite);
    }
    Ok(normalize_heading_deg(mag_deg + dec_deg))
}

/// Compute magnetic heading from true heading and declination (in degrees).
pub fn true_to_magnetic_heading_deg(true_deg: f32, dec_deg: f32) -> Result<f32, DeclinationError> {
    if !true_deg.is_finite() || !dec_deg.is_finite() {
        return Err(DeclinationError::NonFinite);
    }
    Ok(normalize_heading_deg(true_deg - dec_deg))
}

/// Bivariate polynomial model for regional or global declination approximation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeclinationPolynomial {
    pub origin_lat: f32,
    pub origin_lon: f32,
    pub scale_lat: f32,
    pub scale_lon: f32,
    pub coeffs: [f32; 6],
}

impl DeclinationPolynomial {
    pub const fn new(
        origin_lat: f32,
        origin_lon: f32,
        scale_lat: f32,
        scale_lon: f32,
        coeffs: [f32; 6],
    ) -> Self {
        Self {
            origin_lat,
            origin_lon,
            scale_lat,
            scale_lon,
            coeffs,
        }
    }

    pub fn eval(&self, lat_deg: f32, lon_deg: f32) -> Result<f32, DeclinationError> {
        if !lat_deg.is_finite() || !lon_deg.is_finite() {
            return Err(DeclinationError::NonFinite);
        }
        if !(-90.0..=90.0).contains(&lat_deg) {
            return Err(DeclinationError::InvalidLatitude);
        }
        if !(-180.0..=180.0).contains(&lon_deg) {
            return Err(DeclinationError::InvalidLongitude);
        }
        if self.scale_lat == 0.0
            || self.scale_lon == 0.0
            || !self.scale_lat.is_finite()
            || !self.scale_lon.is_finite()
        {
            return Err(DeclinationError::InvalidScale);
        }
        let u = (lat_deg - self.origin_lat) / self.scale_lat;
        let v = (lon_deg - self.origin_lon) / self.scale_lon;
        let [c0, c1, c2, c3, c4, c5] = self.coeffs;
        Ok(c0 + c1 * u + c2 * v + c3 * u * u + c4 * u * v + c5 * v * v)
    }
}

/// Default global reference polynomial for approximate magnetic declination.
pub const WMM_GLOBAL_REF: DeclinationPolynomial =
    DeclinationPolynomial::new(0.0, 0.0, 90.0, 180.0, [-3.5, 4.2, -8.1, -1.8, 6.4, 2.5]);

/// Compute approximate magnetic declination in degrees at (lat_deg, lon_deg).
#[inline]
pub fn wmm_declination_deg(lat_deg: f32, lon_deg: f32) -> Result<f32, DeclinationError> {
    if !lat_deg.is_finite() || !lon_deg.is_finite() {
        return Err(DeclinationError::NonFinite);
    }
    if !(-90.0..=90.0).contains(&lat_deg) {
        return Err(DeclinationError::InvalidLatitude);
    }
    if !(-180.0..=180.0).contains(&lon_deg) {
        return Err(DeclinationError::InvalidLongitude);
    }
    Ok(eval_wmm_field(lat_deg, lon_deg))
}

/// Compute approximate magnetic declination in radians at (lat_rad, lon_rad).
pub fn wmm_declination_rad(lat_rad: f32, lon_rad: f32) -> Result<f32, DeclinationError> {
    let rad_to_deg = 180.0 / core::f32::consts::PI;
    let deg_to_rad = core::f32::consts::PI / 180.0;
    let dec_deg = wmm_declination_deg(lat_rad * rad_to_deg, lon_rad * rad_to_deg)?;
    Ok(dec_deg * deg_to_rad)
}
