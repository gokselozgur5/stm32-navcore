//! Inertial navigation dead reckoning position and velocity propagation.

use crate::geo::GeoError;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DeadReckoningState {
    pub lat_rad: f64,
    pub lon_rad: f64,
    pub height_m: f64,
    pub v_n: f64,
    pub v_e: f64,
    pub v_d: f64,
}

impl DeadReckoningState {
    pub fn new(lat: f64, lon: f64, h: f64, vn: f64, ve: f64, vd: f64) -> Self {
        Self { lat_rad: lat, lon_rad: lon, height_m: h, v_n: vn, v_e: ve, v_d: vd }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NavError {
    InvalidState,
    NonFinite,
    Geo(GeoError),
}

use crate::geo::earth_rate::principal_radii_of_curvature;

/// Propagates geodetic position and height under constant velocity over interval dt.
pub fn dead_reckoning_step(
    state: &DeadReckoningState,
    dt_s: f64,
) -> Result<DeadReckoningState, NavError> {
    if !dt_s.is_finite() || dt_s < 0.0 {
        return Err(NavError::NonFinite);
    }
    if !state.lat_rad.is_finite() || !state.lon_rad.is_finite() || !state.height_m.is_finite() {
        return Err(NavError::NonFinite);
    }

    if dt_s == 0.0 || (state.v_n == 0.0 && state.v_e == 0.0 && state.v_d == 0.0) {
        return Ok(*state);
    }
    let (r_m, r_n) = principal_radii_of_curvature(state.lat_rad)
        .map_err(NavError::Geo)?;
