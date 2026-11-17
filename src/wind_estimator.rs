//! Wind vector triangle estimator from ground speed, TAS, and heading.
//!
//! Solves the aeronautical wind triangle relationship:
//! `V_ground = V_air + W_wind` => `W_wind = V_ground - V_air`.

use core::f32::consts::PI;

/// Errors returned by wind estimator operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WindEstimatorError {
    /// Non-finite input value (NaN or infinity).
    NonFinite,
    /// Airspeed is negative.
    NegativeAirspeed,
    /// Ground speed is negative.
    NegativeGroundSpeed,
    /// Airspeed is below minimum threshold for reliable estimation.
    AirspeedTooLow,
