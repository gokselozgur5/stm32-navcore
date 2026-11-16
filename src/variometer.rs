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
