//! Barometric vertical speed and variometer estimator.

/// Errors returned by variometer operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum VariometerError {
    /// Non-finite input (NaN or infinite).
    NonFinite,
    /// Sample interval must be positive and finite.
    InvalidSampleInterval,
    /// Pressure must be positive and finite.
    InvalidPressure,
    /// Time constant must be strictly positive and finite.
    InvalidTimeConstant,
    /// Threshold values must be finite and consistent (climb >= sink).
    InvalidThreshold,
}

/// Factor to convert meters per second to feet per minute (60 / 0.3048).
pub const MPS_TO_FPM: f32 = 196.85039;

/// Factor to convert meters per second to knots (1 / 0.514444).
pub const MPS_TO_KNOTS: f32 = 1.9438445;

/// Converts vertical speed from meters per second to feet per minute.
#[inline]
pub fn mps_to_fpm(mps: f32) -> f32 {
    mps * MPS_TO_FPM
}

/// Converts vertical speed from feet per minute to meters per second.
#[inline]
pub fn fpm_to_mps(fpm: f32) -> f32 {
    fpm / MPS_TO_FPM
}

/// Converts vertical speed from meters per second to knots.
#[inline]
pub fn mps_to_knots(mps: f32) -> f32 {
    mps * MPS_TO_KNOTS
}

/// Converts vertical speed from knots to meters per second.
#[inline]
pub fn knots_to_mps(knots: f32) -> f32 {
    knots / MPS_TO_KNOTS
}

/// Computes total energy compensated altitude: H_te = H_static + (V^2 / (2 * g0)).
pub fn total_energy_altitude(static_alt_m: f32, tas_mps: f32) -> Result<f32, VariometerError> {
    if !static_alt_m.is_finite() || !tas_mps.is_finite() {
        return Err(VariometerError::NonFinite);
    }
    const G0: f32 = 9.80665;
    let kinetic_head = (tas_mps * tas_mps) / (2.0 * G0);
    Ok(static_alt_m + kinetic_head)
}

/// Audio tone indicator state of the variometer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum VarioTone {
    /// In deadband between sink and climb thresholds (silent).
    Silent,
    /// Climbing above threshold with tone frequency in Hz.
    Climb {
        /// Audio beep frequency in Hz.
        freq_hz: f32,
    },
    /// Sinking below threshold with tone frequency in Hz.
    Sink {
        /// Continuous tone frequency in Hz.
        freq_hz: f32,
    },
}

/// Configuration parameters for the variometer estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VariometerConfig {
    /// Low-pass filter time constant for vertical speed in seconds.
    pub time_constant_s: f32,
    /// Minimum climb rate in m/s to trigger climb indication.
    pub climb_threshold_mps: f32,
    /// Maximum sink rate in m/s (negative) to trigger sink indication.
    pub sink_threshold_mps: f32,
    /// Base audio frequency in Hz for climb tones.
    pub base_freq_hz: f32,
}

impl Default for VariometerConfig {
    fn default() -> Self {
        Self {
            time_constant_s: 1.5,
            climb_threshold_mps: 0.15,
            sink_threshold_mps: -0.5,
            base_freq_hz: 440.0,
        }
    }
}

/// Real-time barometric vertical speed indicator (VSI) and variometer.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Variometer {
    config: VariometerConfig,
    raw_altitude_m: f32,
    filtered_altitude_m: f32,
    vertical_speed_mps: f32,
    initialized: bool,
}

impl Variometer {
    /// Creates a new variometer estimator with given configuration.
    pub fn new(config: VariometerConfig) -> Result<Self, VariometerError> {
        if !config.time_constant_s.is_finite() || config.time_constant_s <= 0.0 {
            return Err(VariometerError::InvalidTimeConstant);
        }
        if !config.climb_threshold_mps.is_finite()
            || !config.sink_threshold_mps.is_finite()
            || config.climb_threshold_mps < config.sink_threshold_mps
        {
            return Err(VariometerError::InvalidThreshold);
        }
        Ok(Self {
            config,
            raw_altitude_m: 0.0,
            filtered_altitude_m: 0.0,
            vertical_speed_mps: 0.0,
            initialized: false,
        })
    }
}

impl Default for Variometer {
    fn default() -> Self {
        Self::new(VariometerConfig::default()).expect("default config is valid")
    }
}

impl Variometer {
    /// Resets estimator state to uninitialized zeros.
    pub fn reset(&mut self) {
        self.raw_altitude_m = 0.0;
        self.filtered_altitude_m = 0.0;
        self.vertical_speed_mps = 0.0;
        self.initialized = false;
    }

    /// Returns a reference to the active configuration.
    pub const fn config(&self) -> &VariometerConfig {
        &self.config
    }
}

impl Variometer {
    /// Updates estimator with measured altitude in meters and timestep in seconds.
    pub fn update_altitude(&mut self, altitude_m: f32, dt_s: f32) -> Result<f32, VariometerError> {
        if !altitude_m.is_finite() {
            return Err(VariometerError::NonFinite);
        }
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(VariometerError::InvalidSampleInterval);
        }
        if !self.initialized {
            self.raw_altitude_m = altitude_m;
            self.filtered_altitude_m = altitude_m;
            self.vertical_speed_mps = 0.0;
            self.initialized = true;
        } else {
            let raw_vs = (altitude_m - self.raw_altitude_m) / dt_s;
            self.raw_altitude_m = altitude_m;
            let alpha = dt_s / (self.config.time_constant_s + dt_s);
            self.vertical_speed_mps += alpha * (raw_vs - self.vertical_speed_mps);
            self.filtered_altitude_m +=
                self.vertical_speed_mps * dt_s + alpha * (altitude_m - self.filtered_altitude_m);
        }
        Ok(self.vertical_speed_mps)
    }
}

impl Variometer {
    /// Updates estimator with measured pressure in Pascals using standard ISA atmosphere.
    pub fn update_pressure(&mut self, pressure_pa: f32, dt_s: f32) -> Result<f32, VariometerError> {
        if !pressure_pa.is_finite() {
            return Err(VariometerError::NonFinite);
        }
        if pressure_pa <= 0.0 {
            return Err(VariometerError::InvalidPressure);
        }
        let alt_m = crate::baro_alt::altitude_from_pressure_isa(pressure_pa)
            .map_err(|_| VariometerError::InvalidPressure)?;
        self.update_altitude(alt_m, dt_s)
    }
}

impl Variometer {
    /// Updates estimator with measured pressure and local QNH in Pascals.
    pub fn update_pressure_qnh(
        &mut self,
        pressure_pa: f32,
        qnh_pa: f32,
        dt_s: f32,
    ) -> Result<f32, VariometerError> {
        if !pressure_pa.is_finite() || !qnh_pa.is_finite() {
            return Err(VariometerError::NonFinite);
        }
        if pressure_pa <= 0.0 || qnh_pa <= 0.0 {
            return Err(VariometerError::InvalidPressure);
        }
        let alt_m = crate::baro_alt::altitude_from_pressure_qnh(pressure_pa, qnh_pa)
            .map_err(|_| VariometerError::InvalidPressure)?;
        self.update_altitude(alt_m, dt_s)
    }
}

impl Variometer {
    /// Returns filtered altitude in meters.
    pub const fn altitude(&self) -> f32 {
        self.filtered_altitude_m
    }

    /// Returns most recent raw altitude in meters.
    pub const fn raw_altitude(&self) -> f32 {
        self.raw_altitude_m
    }

    /// Returns estimated vertical speed in meters per second.
    pub const fn vertical_speed(&self) -> f32 {
        self.vertical_speed_mps
    }
}

impl Variometer {
    /// Returns estimated vertical speed in feet per minute.
    pub fn vertical_speed_fpm(&self) -> f32 {
        mps_to_fpm(self.vertical_speed_mps)
    }

    /// Returns estimated vertical speed in knots.
    pub fn vertical_speed_knots(&self) -> f32 {
        mps_to_knots(self.vertical_speed_mps)
    }

    /// Returns true if the estimator has received at least one sample.
    pub const fn is_initialized(&self) -> bool {
        self.initialized
    }
}

impl Variometer {
    /// Determines current audio variometer tone state and frequency.
    pub fn tone(&self) -> VarioTone {
        if self.vertical_speed_mps >= self.config.climb_threshold_mps {
            let excess = self.vertical_speed_mps - self.config.climb_threshold_mps;
            let freq_hz = self.config.base_freq_hz + excess * 150.0;
            VarioTone::Climb { freq_hz }
        } else if self.vertical_speed_mps <= self.config.sink_threshold_mps {
            let excess = self.config.sink_threshold_mps - self.vertical_speed_mps;
            let freq_hz = (self.config.base_freq_hz * 0.75 - excess * 50.0).max(100.0);
            VarioTone::Sink { freq_hz }
        } else {
            VarioTone::Silent
        }
    }
}
