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
}
