//! Hypsometric barometric altitude estimator.

/// Errors that can occur during barometric altitude estimation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BaroAltError {
    NonFinite,
    InvalidPressure,
    InvalidReferencePressure,
    InvalidTemperature,
    InvalidLapseRate,
}

/// Standard sea-level atmospheric pressure in Pascals (1013.25 hPa).
pub const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0;

/// Standard sea-level temperature in Kelvin (15.0 deg C).
pub const STANDARD_SEA_LEVEL_TEMP_K: f32 = 288.15;

/// Standard troposphere temperature lapse rate in K/m.
pub const STANDARD_LAPSE_RATE_K_PER_M: f32 = 0.0065;

/// Standard acceleration due to gravity in m/s^2.
pub const STANDARD_GRAVITY_M_S2: f32 = 9.80665;

/// Specific gas constant for dry air in J/(kg*K).
pub const GAS_CONSTANT_DRY_AIR: f32 = 287.05287;

/// Hypsometric exponent for standard atmosphere: (R * L) / g0.
pub const HYPSOMETRIC_EXPONENT: f32 =
    (GAS_CONSTANT_DRY_AIR * STANDARD_LAPSE_RATE_K_PER_M) / STANDARD_GRAVITY_M_S2;

/// Standard troposphere temperature scale height factor: T0 / L.
pub const TROPOSPHERE_TEMP_SCALE: f32 =
    STANDARD_SEA_LEVEL_TEMP_K / STANDARD_LAPSE_RATE_K_PER_M;

/// Computes altitude in meters above sea level from pressure under standard ISA conditions.
pub fn altitude_from_pressure_isa(pressure_pa: f32) -> Result<f32, BaroAltError> {
    if !pressure_pa.is_finite() {
        return Err(BaroAltError::NonFinite);
    }
    if pressure_pa <= 0.0 {
        return Err(BaroAltError::InvalidPressure);
    }
    let ratio = pressure_pa / STANDARD_SEA_LEVEL_PRESSURE_PA;
    let factor = 1.0 - libm::powf(ratio, HYPSOMETRIC_EXPONENT);
    Ok(TROPOSPHERE_TEMP_SCALE * factor)
}

/// Computes altitude in meters above sea level given measured pressure and local QNH.
pub fn altitude_from_pressure_qnh(
    pressure_pa: f32,
    qnh_pa: f32,
) -> Result<f32, BaroAltError> {
    if !pressure_pa.is_finite() || !qnh_pa.is_finite() {
        return Err(BaroAltError::NonFinite);
    }
    if pressure_pa <= 0.0 {
        return Err(BaroAltError::InvalidPressure);
    }
    if qnh_pa <= 0.0 {
        return Err(BaroAltError::InvalidReferencePressure);
    }
    let ratio = pressure_pa / qnh_pa;
    let factor = 1.0 - libm::powf(ratio, HYPSOMETRIC_EXPONENT);
    Ok(TROPOSPHERE_TEMP_SCALE * factor)
}

fn validate_baro_inputs(
    p: f32,
    p_ref: f32,
    t_ref: f32,
    lapse: f32,
) -> Result<(), BaroAltError> {
    if !p.is_finite() || !p_ref.is_finite() || !t_ref.is_finite() || !lapse.is_finite() {
        return Err(BaroAltError::NonFinite);
    }
    if p <= 0.0 {
        return Err(BaroAltError::InvalidPressure);
    }
    if p_ref <= 0.0 {
        return Err(BaroAltError::InvalidReferencePressure);
    }
    if t_ref <= 0.0 {
        return Err(BaroAltError::InvalidTemperature);
    }
    if lapse < 0.0 {
        return Err(BaroAltError::InvalidLapseRate);
    }
    Ok(())
}
