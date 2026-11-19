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

/// Calculate instantaneous wind vector [North, East] from ground speed, course track, TAS, and heading.
pub fn instantaneous_wind_from_course(
    ground_speed_mps: f32,
    course_rad: f32,
    tas_mps: f32,
    heading_rad: f32,
) -> Result<[f32; 2], WindEstimatorError> {
    if !ground_speed_mps.is_finite()
        || !course_rad.is_finite()
        || !tas_mps.is_finite()
        || !heading_rad.is_finite()
    {
        return Err(WindEstimatorError::NonFinite);
    }
    if ground_speed_mps < 0.0 {
        return Err(WindEstimatorError::NegativeGroundSpeed);
    }
    if tas_mps < 0.0 {
        return Err(WindEstimatorError::NegativeAirspeed);
    }
    let vg_n = ground_speed_mps * libm::cosf(course_rad);
    let vg_e = ground_speed_mps * libm::sinf(course_rad);
    instantaneous_wind_vector([vg_n, vg_e], tas_mps, heading_rad)
}

/// Calculate wind speed and meteorological/trajectory directions from wind vector.
pub fn wind_speed_and_direction(wind_ne_mps: [f32; 2]) -> Result<WindVectorInfo, WindEstimatorError> {
    if !wind_ne_mps[0].is_finite() || !wind_ne_mps[1].is_finite() {
        return Err(WindEstimatorError::NonFinite);
    }
    let wn = wind_ne_mps[0];
    let we = wind_ne_mps[1];
    let speed = libm::sqrtf(wn * wn + we * we);
    let to_rad = wrap_2pi(libm::atan2f(we, wn));
    let to_deg = to_rad * (180.0 / PI);
    let from_rad = wrap_2pi(libm::atan2f(-we, -wn));
    let from_deg = from_rad * (180.0 / PI);
    Ok(WindVectorInfo {
        speed_mps: speed,
        direction_from_rad: from_rad,
        direction_from_deg: from_deg,
        direction_to_rad: to_rad,
        direction_to_deg: to_deg,
    })
}

/// Decompose wind vector into headwind and crosswind relative to aircraft heading.
pub fn decompose_relative_wind(
    wind_ne_mps: [f32; 2],
    heading_rad: f32,
) -> Result<RelativeWind, WindEstimatorError> {
    if !wind_ne_mps[0].is_finite() || !wind_ne_mps[1].is_finite() || !heading_rad.is_finite() {
        return Err(WindEstimatorError::NonFinite);
    }
    let wn = wind_ne_mps[0];
    let we = wind_ne_mps[1];
    let cos_psi = libm::cosf(heading_rad);
    let sin_psi = libm::sinf(heading_rad);
    let headwind_mps = -(wn * cos_psi + we * sin_psi);
    let crosswind_mps = wn * sin_psi - we * cos_psi;
    Ok(RelativeWind {
        headwind_mps,
        crosswind_mps,
    })
}

/// Calculate wind correction angle (drift angle) in radians: `WCA = course - heading`.
pub fn wind_correction_angle_rad(course_rad: f32, heading_rad: f32) -> Result<f32, WindEstimatorError> {
    if !course_rad.is_finite() || !heading_rad.is_finite() {
        return Err(WindEstimatorError::NonFinite);
    }
    Ok(wrap_pi(course_rad - heading_rad))
}

/// Stateful recursive low-pass wind vector estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WindEstimator {
    config: WindEstimatorConfig,
    wind_ne_mps: [f32; 2],
    initialized: bool,
    sample_count: u32,
}

impl WindEstimator {
    /// Create new wind estimator with given configuration.
    pub fn new(config: WindEstimatorConfig) -> Result<Self, WindEstimatorError> {
        config.validate()?;
        Ok(Self {
            config,
            wind_ne_mps: [0.0, 0.0],
            initialized: false,
            sample_count: 0,
        })
    }

    /// Reset estimator internal filter state.
    pub fn reset(&mut self) {
        self.wind_ne_mps = [0.0, 0.0];
        self.initialized = false;
        self.sample_count = 0;
    }

    /// Current configuration reference.
    pub fn config(&self) -> &WindEstimatorConfig {
        &self.config
    }
    /// Check if estimator has received at least one valid measurement.
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// Total valid updates processed since creation or reset.
    pub fn sample_count(&self) -> u32 {
        self.sample_count
    }

    /// Get current filtered wind vector [North, East] in m/s.
    pub fn wind_vector(&self) -> [f32; 2] {
        self.wind_ne_mps
    }

    /// Get current filtered wind speed in m/s.
    pub fn wind_speed(&self) -> f32 {
        let [wn, we] = self.wind_ne_mps;
        libm::sqrtf(wn * wn + we * we)
    }

    /// Get current meteorological wind direction (FROM) in radians [0, 2pi).
    pub fn wind_direction_from_rad(&self) -> f32 {
        let [wn, we] = self.wind_ne_mps;
        wrap_2pi(libm::atan2f(-we, -wn))
    }
    /// Get current meteorological wind direction (FROM) in degrees [0, 360).
    pub fn wind_direction_from_deg(&self) -> f32 {
        self.wind_direction_from_rad() * (180.0 / PI)
    }

    /// Get current relative wind decomposition against a heading.
    pub fn relative_wind(&self, heading_rad: f32) -> Result<RelativeWind, WindEstimatorError> {
        decompose_relative_wind(self.wind_ne_mps, heading_rad)
    }
    /// Update filter state from ground velocity vector, TAS, and heading with time delta.
    pub fn update(
        &mut self,
        ground_vel_ne: [f32; 2],
        tas_mps: f32,
        heading_rad: f32,
        dt_s: f32,
    ) -> Result<WindEstimate, WindEstimatorError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(WindEstimatorError::InvalidSampleInterval);
        }
        let inst_w = instantaneous_wind_vector(ground_vel_ne, tas_mps, heading_rad)?;
        if tas_mps < self.config.min_airspeed_mps {
            return Err(WindEstimatorError::AirspeedTooLow);
        }
        let inst_speed = libm::sqrtf(inst_w[0] * inst_w[0] + inst_w[1] * inst_w[1]);
        if inst_speed > self.config.max_wind_speed_mps {
            return Err(WindEstimatorError::WindSpeedExceeded);
        }
        if !self.initialized {
            self.wind_ne_mps = inst_w;
            self.initialized = true;
        } else {
            let alpha = dt_s / (self.config.time_constant_s + dt_s);
            self.wind_ne_mps[0] += alpha * (inst_w[0] - self.wind_ne_mps[0]);
            self.wind_ne_mps[1] += alpha * (inst_w[1] - self.wind_ne_mps[1]);
        }
        self.sample_count = self.sample_count.saturating_add(1);
        let speed = self.wind_speed();
        let from_rad = self.wind_direction_from_rad();
        Ok(WindEstimate {
            wind_ne_mps: self.wind_ne_mps,
            speed_mps: speed,
            direction_from_rad: from_rad,
            direction_from_deg: from_rad * (180.0 / PI),
        })
    }
    /// Update filter state from ground speed, course track, TAS, and heading with time delta.
    pub fn update_from_course(
        &mut self,
        ground_speed_mps: f32,
        course_rad: f32,
        tas_mps: f32,
        heading_rad: f32,
        dt_s: f32,
    ) -> Result<WindEstimate, WindEstimatorError> {
        if !ground_speed_mps.is_finite() || !course_rad.is_finite() {
            return Err(WindEstimatorError::NonFinite);
        }
        if ground_speed_mps < 0.0 {
            return Err(WindEstimatorError::NegativeGroundSpeed);
        }
        let vg_n = ground_speed_mps * libm::cosf(course_rad);
        let vg_e = ground_speed_mps * libm::sinf(course_rad);
        self.update([vg_n, vg_e], tas_mps, heading_rad, dt_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_wrap_angles() {
        assert_eq!(wrap_2pi(0.0), 0.0);
        assert!((wrap_2pi(2.0 * PI) - 0.0).abs() < 1e-6);
        assert!((wrap_2pi(-PI / 2.0) - 3.0 * PI / 2.0).abs() < 1e-6);
        assert!((wrap_pi(PI).abs() - PI).abs() < 1e-6);
        assert!((wrap_pi(3.0 * PI).abs() - PI).abs() < 1e-6);
        assert!((wrap_pi(-3.0 * PI).abs() - PI).abs() < 1e-6);
    }
    #[test]
    fn test_instantaneous_wind_vector() {
        // Aircraft flying North at 25 m/s TAS, ground velocity North 30 m/s => tailwind 5 m/s
        let res = instantaneous_wind_vector([30.0, 0.0], 25.0, 0.0).unwrap();
        assert!((res[0] - 5.0).abs() < 1e-5);
        assert!(res[1].abs() < 1e-5);

        // Aircraft heading East at 20 m/s TAS, ground speed 20 m/s East, 5 m/s North
        let res2 = instantaneous_wind_vector([5.0, 20.0], 20.0, PI / 2.0).unwrap();
        assert!((res2[0] - 5.0).abs() < 1e-5);
        assert!(res2[1].abs() < 1e-5);
    }
    #[test]
    fn test_wind_directions_and_relative() {
        // Wind blowing North at 10 m/s => comes FROM South (180 deg)
        let info = wind_speed_and_direction([10.0, 0.0]).unwrap();
        assert!((info.speed_mps - 10.0).abs() < 1e-5);
        assert!((info.direction_from_deg - 180.0).abs() < 1e-3);
        assert!((info.direction_to_deg - 0.0).abs() < 1e-3);

        // Heading North, wind blowing North (tailwind)
        let rel = decompose_relative_wind([10.0, 0.0], 0.0).unwrap();
        assert!((rel.headwind_mps - (-10.0)).abs() < 1e-5);
        assert!(rel.crosswind_mps.abs() < 1e-5);
    }
    #[test]
    fn test_wind_estimator_filter_convergence() {
        let cfg = WindEstimatorConfig {
            time_constant_s: 2.0,
            min_airspeed_mps: 5.0,
            max_wind_speed_mps: 40.0,
        };
        let mut est = WindEstimator::new(cfg).unwrap();
        // Step with steady 10 m/s East wind: TAS 20 m/s North, ground vel [20, 10]
        for _ in 0..50 {
            let _ = est.update([20.0, 10.0], 20.0, 0.0, 0.1).unwrap();
        }
        let w = est.wind_vector();
        assert!(w[0].abs() < 0.1);
        assert!((w[1] - 10.0).abs() < 0.1);
    }
}
