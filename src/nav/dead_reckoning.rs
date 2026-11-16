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
