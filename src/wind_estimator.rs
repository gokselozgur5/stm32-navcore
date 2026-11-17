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

/// Configuration for recursive wind vector estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindEstimatorConfig {
    /// Filter time constant tau in seconds for exponential smoothing.
    pub time_constant_s: f32,
    /// Minimum airspeed in m/s required to update wind state.
    pub min_airspeed_mps: f32,
    /// Maximum plausible wind speed in m/s to gate outliers.
    pub max_wind_speed_mps: f32,
}

impl WindEstimatorConfig {
    /// Validate configuration values.
    pub fn validate(&self) -> Result<(), WindEstimatorError> {
        if !self.time_constant_s.is_finite() || self.time_constant_s <= 0.0 {
            return Err(WindEstimatorError::InvalidTimeConstant);
        }
        if !self.min_airspeed_mps.is_finite() || self.min_airspeed_mps < 0.0 {
            return Err(WindEstimatorError::InvalidConfig);
        }
        if !self.max_wind_speed_mps.is_finite() || self.max_wind_speed_mps <= 0.0 {
            return Err(WindEstimatorError::InvalidConfig);
        }
        if self.min_airspeed_mps >= self.max_wind_speed_mps {
            return Err(WindEstimatorError::InvalidConfig);
        }
        Ok(())
    }
}

impl Default for WindEstimatorConfig {
    fn default() -> Self {
        Self {
            time_constant_s: 15.0,
            min_airspeed_mps: 3.0,
            max_wind_speed_mps: 60.0,
        }
    }
}

/// Wrap angle into [0, 2*pi) radians.
pub fn wrap_2pi(angle_rad: f32) -> f32 {
    let two_pi = 2.0 * PI;
    let mut wrapped = angle_rad % two_pi;
    if wrapped < 0.0 {
        wrapped += two_pi;
    }
    wrapped
}

/// Wrap angle into [-pi, pi] radians.
pub fn wrap_pi(angle_rad: f32) -> f32 {
    let two_pi = 2.0 * PI;
    let mut wrapped = (angle_rad + PI) % two_pi;
    if wrapped < 0.0 {
        wrapped += two_pi;
    }
    wrapped - PI
}

/// Calculate instantaneous wind vector [North, East] in m/s from ground velocity and airspeed/heading.
pub fn instantaneous_wind_vector(
    ground_vel_ne: [f32; 2],
    tas_mps: f32,
    heading_rad: f32,
) -> Result<[f32; 2], WindEstimatorError> {
    if !ground_vel_ne[0].is_finite()
        || !ground_vel_ne[1].is_finite()
        || !tas_mps.is_finite()
        || !heading_rad.is_finite()
    {
        return Err(WindEstimatorError::NonFinite);
    }
    if tas_mps < 0.0 {
        return Err(WindEstimatorError::NegativeAirspeed);
    }
    let v_air_n = tas_mps * libm::cosf(heading_rad);
    let v_air_e = tas_mps * libm::sinf(heading_rad);
    let w_n = ground_vel_ne[0] - v_air_n;
    let w_e = ground_vel_ne[1] - v_air_e;
    Ok([w_n, w_e])
}
