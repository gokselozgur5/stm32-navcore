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
    /// Estimated wind speed exceeds maximum physical plausibility limit.
    WindSpeedExceeded,
    /// Sample time interval is non-positive or non-finite.
    InvalidSampleInterval,
    /// Filter time constant is non-positive or non-finite.
    InvalidTimeConstant,
    /// Configuration parameter is out of valid range.
    InvalidConfig,
}

/// Wind vector speed and direction representation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindVectorInfo {
    /// Wind speed magnitude in m/s.
    pub speed_mps: f32,
    /// Meteorological wind direction (FROM which wind blows) in radians [0, 2pi).
    pub direction_from_rad: f32,
    /// Meteorological wind direction in degrees [0, 360).
    pub direction_from_deg: f32,
    /// Direction towards which wind blows in radians [0, 2pi).
    pub direction_to_rad: f32,
    /// Direction towards which wind blows in degrees [0, 360).
    pub direction_to_deg: f32,
}

/// Relative wind components resolved into the aircraft heading frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RelativeWind {
    /// Headwind component in m/s (positive = headwind, negative = tailwind).
    pub headwind_mps: f32,
    /// Crosswind component in m/s (positive = from starboard, negative = from port).
    pub crosswind_mps: f32,
}

/// Filtered wind estimate containing vector and derived quantities.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindEstimate {
    /// Wind velocity vector [North, East] in m/s.
    pub wind_ne_mps: [f32; 2],
    /// Wind speed magnitude in m/s.
    pub speed_mps: f32,
    /// Direction FROM which wind blows in radians [0, 2pi).
    pub direction_from_rad: f32,
    /// Direction FROM which wind blows in degrees [0, 360).
    pub direction_from_deg: f32,
}
