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
    /// Pitot calibration factor is non-positive or non-finite.
    InvalidFactor,
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
pub fn ias_to_tas(ias_mps: f32, static_pressure_pa: f32, temp_k: f32) -> Result<f32, PitotError> {
    let rho = air_density(static_pressure_pa, temp_k)?;
    ias_to_tas_with_density(ias_mps, rho)
}

/// Convert pitot differential (dynamic) pressure in Pa to indicated airspeed (IAS) in m/s.
pub fn diff_pressure_to_ias(diff_pressure_pa: f32) -> Result<f32, PitotError> {
    if !diff_pressure_pa.is_finite() {
        return Err(PitotError::NonFinite);
    }
    if diff_pressure_pa < 0.0 {
        return Err(PitotError::NegativeDifferentialPressure);
    }
    let v_sq = (2.0 * diff_pressure_pa) / STANDARD_SEA_LEVEL_DENSITY_KG_M3;
    Ok(libm::sqrtf(v_sq))
}

/// Convert pitot differential pressure in Pa directly to true airspeed (TAS) in m/s.
pub fn diff_pressure_to_tas(
    diff_pressure_pa: f32,
    static_pressure_pa: f32,
    temp_k: f32,
) -> Result<f32, PitotError> {
    if !diff_pressure_pa.is_finite() {
        return Err(PitotError::NonFinite);
    }
    if diff_pressure_pa < 0.0 {
        return Err(PitotError::NegativeDifferentialPressure);
    }
    crate::airdata::airspeed::true_airspeed(diff_pressure_pa, static_pressure_pa, temp_k).map_err(
        |e| match e {
            crate::airdata::airspeed::AirspeedError::NonFinite => PitotError::NonFinite,
            crate::airdata::airspeed::AirspeedError::NegativeDifferentialPressure => {
                PitotError::NegativeDifferentialPressure
            }
            crate::airdata::airspeed::AirspeedError::InvalidPressure => PitotError::InvalidPressure,
            _ => PitotError::InvalidTemperature,
        },
    )
}

/// Complete airspeed telemetry reading including IAS, TAS, air density, and dynamic pressure.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirspeedReading {
    /// Indicated airspeed in m/s.
    pub ias_mps: f32,
    /// True airspeed in m/s.
    pub tas_mps: f32,
    /// Calculated ambient air density in kg/m^3.
    pub density_kg_m3: f32,
    /// Dynamic (pitot differential) pressure in Pa.
    pub dynamic_pressure_pa: f32,
}

/// Pitot-static true airspeed estimator maintaining calibration parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PitotTasEstimator {
    /// Pitot tube installation calibration factor (default 1.0).
    pub pitot_factor: f32,
}

impl PitotTasEstimator {
    /// Create new estimator with nominal unity pitot factor.
    pub const fn new() -> Self {
        Self { pitot_factor: 1.0 }
    }

    /// Create new estimator with custom calibration factor.
    pub fn with_factor(pitot_factor: f32) -> Result<Self, PitotError> {
        if !pitot_factor.is_finite() || pitot_factor <= 0.0 {
            return Err(PitotError::InvalidFactor);
        }
        Ok(Self { pitot_factor })
    }
}

impl PitotTasEstimator {
    /// Update airspeed estimation from pitot differential pressure, static pressure, and temperature.
    pub fn update(
        &self,
        diff_pressure_pa: f32,
        static_pressure_pa: f32,
        temp_k: f32,
    ) -> Result<AirspeedReading, PitotError> {
        let cal_diff = diff_pressure_pa * self.pitot_factor;
        let ias = diff_pressure_to_ias(cal_diff)?;
        let rho = air_density(static_pressure_pa, temp_k)?;
        let tas = ias_to_tas_with_density(ias, rho)?;
        Ok(AirspeedReading {
            ias_mps: ias,
            tas_mps: tas,
            density_kg_m3: rho,
            dynamic_pressure_pa: cal_diff,
        })
    }
}

impl Default for PitotTasEstimator {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert temperature from Celsius to Kelvin.
pub fn celsius_to_kelvin(temp_c: f32) -> Result<f32, PitotError> {
    if !temp_c.is_finite() {
        return Err(PitotError::NonFinite);
    }
    let k = temp_c + 273.15;
    if k <= 0.0 {
        return Err(PitotError::InvalidTemperature);
    }
    Ok(k)
}
