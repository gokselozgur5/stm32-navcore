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
