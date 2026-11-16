//! True airspeed (TAS) estimation from pitot differential pressure and indicated airspeed (IAS).

/// Errors in pitot and airspeed calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PitotError {
    /// Input value is NaN or infinite.
    NonFinite,
    /// Static pressure is non-positive or unphysical.
    InvalidPressure,
    /// Absolute temperature is non-positive or below zero Kelvin.
    InvalidTemperature,
    /// Differential pressure is negative.
    NegativeDifferentialPressure,
    /// Indicated airspeed is negative.
    NegativeAirspeed,
    /// Calculated density is non-positive or non-finite.
    InvalidDensity,
}

/// Standard sea-level atmospheric pressure in Pascals (1013.25 hPa).
pub const STANDARD_SEA_LEVEL_PRESSURE_PA: f32 = 101325.0;

/// Standard sea-level temperature in Kelvin (15.0 deg C).
pub const STANDARD_SEA_LEVEL_TEMP_K: f32 = 288.15;

/// Specific gas constant for dry air in J/(kg*K).
pub const GAS_CONSTANT_DRY_AIR: f32 = 287.05287;

/// Standard atmospheric air density at sea level in kg/m^3.
pub const STANDARD_SEA_LEVEL_DENSITY_KG_M3: f32 = 1.2250;

fn validate_air_state(static_pressure_pa: f32, temp_k: f32) -> Result<(), PitotError> {
    if !static_pressure_pa.is_finite() || !temp_k.is_finite() {
        return Err(PitotError::NonFinite);
    }
    if static_pressure_pa <= 0.0 {
        return Err(PitotError::InvalidPressure);
    }
    if temp_k <= 0.0 {
        return Err(PitotError::InvalidTemperature);
    }
    Ok(())
}

/// Compute local air density in kg/m^3 using ideal gas equation: rho = P / (R * T).
pub fn air_density(static_pressure_pa: f32, temp_k: f32) -> Result<f32, PitotError> {
    validate_air_state(static_pressure_pa, temp_k)?;
    let rho = static_pressure_pa / (GAS_CONSTANT_DRY_AIR * temp_k);
    if !rho.is_finite() || rho <= 0.0 {
        return Err(PitotError::InvalidDensity);
    }
    Ok(rho)
}

/// Compute atmospheric density ratio sigma = rho / rho0.
pub fn density_ratio(static_pressure_pa: f32, temp_k: f32) -> Result<f32, PitotError> {
    let rho = air_density(static_pressure_pa, temp_k)?;
    let sigma = rho / STANDARD_SEA_LEVEL_DENSITY_KG_M3;
    if !sigma.is_finite() || sigma <= 0.0 {
        return Err(PitotError::InvalidDensity);
    }
    Ok(sigma)
}

/// Convert indicated airspeed (IAS) to true airspeed (TAS) given local air density.
pub fn ias_to_tas_with_density(ias_mps: f32, density_kg_m3: f32) -> Result<f32, PitotError> {
    if !ias_mps.is_finite() || !density_kg_m3.is_finite() {
        return Err(PitotError::NonFinite);
    }
    if ias_mps < 0.0 {
        return Err(PitotError::NegativeAirspeed);
    }
    if density_kg_m3 <= 0.0 {
        return Err(PitotError::InvalidDensity);
    }
    let ratio = STANDARD_SEA_LEVEL_DENSITY_KG_M3 / density_kg_m3;
    Ok(ias_mps * libm::sqrtf(ratio))
}

/// Convert indicated airspeed (IAS) to true airspeed (TAS) given static pressure and temperature.
pub fn ias_to_tas(
    ias_mps: f32,
    static_pressure_pa: f32,
    temp_k: f32,
) -> Result<f32, PitotError> {
    let rho = air_density(static_pressure_pa, temp_k)?;
    ias_to_tas_with_density(ias_mps, rho)
}
