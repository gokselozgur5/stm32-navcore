//! Magnetic declination polynomial approximation for embedded navigation.

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
    if rem < 0.0 {
        rem + 360.0
    } else {
        rem
    }
}

/// Normalize an angle in radians into [0.0, 2*PI).
#[inline]
pub fn normalize_heading_rad(rad: f32) -> f32 {
    let two_pi = 2.0 * core::f32::consts::PI;
    let rem = rad % two_pi;
    if rem < 0.0 {
        rem + two_pi
    } else {
        rem
    }
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
        Self { origin_lat, origin_lon, scale_lat, scale_lon, coeffs }
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
        if self.scale_lat == 0.0 || self.scale_lon == 0.0 || !self.scale_lat.is_finite() || !self.scale_lon.is_finite() {
            return Err(DeclinationError::InvalidScale);
        }
        let u = (lat_deg - self.origin_lat) / self.scale_lat;
        let v = (lon_deg - self.origin_lon) / self.scale_lon;
        let [c0, c1, c2, c3, c4, c5] = self.coeffs;
        Ok(c0 + c1 * u + c2 * v + c3 * u * u + c4 * u * v + c5 * v * v)
    }
}

/// Default global reference polynomial for approximate magnetic declination.
pub const WMM_GLOBAL_REF: DeclinationPolynomial = DeclinationPolynomial::new(
    0.0, 0.0, 90.0, 180.0,
    [-3.5, 4.2, -8.1, -1.8, 6.4, 2.5],
);

/// Compute approximate magnetic declination in degrees at (lat_deg, lon_deg).
#[inline]
pub fn wmm_declination_deg(lat_deg: f32, lon_deg: f32) -> Result<f32, DeclinationError> {
    WMM_GLOBAL_REF.eval(lat_deg, lon_deg)
}

/// Compute approximate magnetic declination in radians at (lat_rad, lon_rad).
pub fn wmm_declination_rad(lat_rad: f32, lon_rad: f32) -> Result<f32, DeclinationError> {
    let rad_to_deg = 180.0 / core::f32::consts::PI;
    let deg_to_rad = core::f32::consts::PI / 180.0;
    let dec_deg = wmm_declination_deg(lat_rad * rad_to_deg, lon_rad * rad_to_deg)?;
    Ok(dec_deg * deg_to_rad)
}
