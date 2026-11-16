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
