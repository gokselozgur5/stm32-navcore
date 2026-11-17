//! Barometric altitude hypsometric equation and temperature lapse compensator.

/// Errors occurring in hypsometric and lapse calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum HypsometricError {
    /// Non-finite numeric input (NaN or Inf).
    NonFinite,
    /// Non-positive atmospheric pressure value.
    InvalidPressure,
    /// Absolute temperature below zero Kelvin.
    InvalidTemperature,
    /// Negative or excessive lapse rate.
    InvalidLapseRate,
    /// Division by zero or indeterminate ratio.
    ZeroDivision,
}

/// Standard sea-level pressure in Pascals (1013.25 hPa).
pub const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0;

/// Standard sea-level temperature in Kelvin (15.0 deg C).
pub const STANDARD_SEA_LEVEL_TEMP_K: f32 = 288.15;

/// Standard tropospheric temperature lapse rate in K/m (0.0065 K/m).
pub const STANDARD_LAPSE_RATE_K_PER_M: f32 = 0.0065;

/// Dry adiabatic lapse rate in K/m (~0.00976 K/m).
pub const DRY_ADIABATIC_LAPSE_RATE_K_PER_M: f32 = 0.00976;

/// Specific gas constant for dry air in J/(kg*K).
pub const GAS_CONSTANT_DRY_AIR: f32 = 287.05287;

/// Standard gravitational acceleration in m/s^2.
pub const STANDARD_GRAVITY_M_S2: f32 = 9.80665;

/// Validate pressure and temperature state parameters.
fn validate_state(p: f32, p_ref: f32, temp_k: f32) -> Result<(), HypsometricError> {
    if !p.is_finite() || !p_ref.is_finite() || !temp_k.is_finite() {
        return Err(HypsometricError::NonFinite);
    }
    if p <= 0.0 || p_ref <= 0.0 {
        return Err(HypsometricError::InvalidPressure);
    }
    if temp_k <= 0.0 {
        return Err(HypsometricError::InvalidTemperature);
    }
    Ok(())
}

/// Compute layer thickness between two pressure boundaries using mean virtual temperature.
pub fn hypsometric_thickness(
    p_base_pa: f32,
    p_top_pa: f32,
    mean_temp_k: f32,
) -> Result<f32, HypsometricError> {
    validate_state(p_top_pa, p_base_pa, mean_temp_k)?;
    if p_top_pa > p_base_pa {
        return Err(HypsometricError::InvalidPressure);
    }
    let ratio = p_base_pa / p_top_pa;
    let factor = (GAS_CONSTANT_DRY_AIR * mean_temp_k) / STANDARD_GRAVITY_M_S2;
    Ok(factor * libm::logf(ratio))
}

/// Compute altitude from pressure using reference level and temperature lapse rate.
pub fn hypsometric_altitude(
    pressure_pa: f32,
    ref_pressure_pa: f32,
    ref_temp_k: f32,
    lapse_rate: f32,
) -> Result<f32, HypsometricError> {
    validate_state(pressure_pa, ref_pressure_pa, ref_temp_k)?;
    if !lapse_rate.is_finite() || lapse_rate < 0.0 || lapse_rate > 0.05 {
        return Err(HypsometricError::InvalidLapseRate);
    }
    let ratio = pressure_pa / ref_pressure_pa;
    if lapse_rate < 1e-7 {
        let scale = (GAS_CONSTANT_DRY_AIR * ref_temp_k) / STANDARD_GRAVITY_M_S2;
        return Ok(scale * -libm::logf(ratio));
    }
    let exponent = (GAS_CONSTANT_DRY_AIR * lapse_rate) / STANDARD_GRAVITY_M_S2;
    Ok((ref_temp_k / lapse_rate) * (1.0 - libm::powf(ratio, exponent)))
}

/// Profile describing local reference state and troposphere lapse rate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HypsometricProfile {
    /// Reference atmospheric pressure in Pascals.
    pub ref_pressure_pa: f32,
    /// Reference atmospheric temperature in Kelvin.
    pub ref_temp_k: f32,
    /// Vertical temperature lapse rate in K/m.
    pub lapse_rate_k_per_m: f32,
    /// Reference geopotential altitude in meters.
    pub ref_altitude_m: f32,
}

impl HypsometricProfile {
    /// Create new profile with specified reference level and lapse rate.
    pub fn new(
        p_ref: f32,
        t_ref: f32,
        lapse: f32,
        alt_ref: f32,
    ) -> Result<Self, HypsometricError> {
        validate_state(p_ref, p_ref, t_ref)?;
        if !lapse.is_finite() || lapse < 0.0 || lapse > 0.05 || !alt_ref.is_finite() {
            return Err(HypsometricError::InvalidLapseRate);
        }
        Ok(Self {
            ref_pressure_pa: p_ref,
            ref_temp_k: t_ref,
            lapse_rate_k_per_m: lapse,
            ref_altitude_m: alt_ref,
        })
    }
}

impl HypsometricProfile {
    /// Construct standard ISA sea-level profile.
    pub fn standard_isa() -> Self {
        Self {
            ref_pressure_pa: STANDARD_SEA_LEVEL_PRESSURE_PA,
            ref_temp_k: STANDARD_SEA_LEVEL_TEMP_K,
            lapse_rate_k_per_m: STANDARD_LAPSE_RATE_K_PER_M,
            ref_altitude_m: 0.0,
        }
    }
}

impl Default for HypsometricProfile {
    fn default() -> Self {
        Self::standard_isa()
    }
}
