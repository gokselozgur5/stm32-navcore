//! Pitot-static indicated airspeed (IAS) and true airspeed (TAS) calculator.

/// Errors occurring in airspeed and atmospheric computations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirspeedError {
    /// Non-finite input parameter.
    NonFinite,
    /// Absolute or static pressure is non-positive.
    InvalidPressure,
    /// Absolute temperature is non-positive.
    InvalidTemperature,
    /// Differential pressure is negative.
    NegativeDifferentialPressure,
    /// Calculated or supplied density is non-positive.
    InvalidDensity,
    /// Airspeed is negative.
    NegativeAirspeed,
    /// Subsonic model exceeded (Mach >= 1.0).
    SupersonicFlow,
}

/// Standard sea-level pressure in Pa.
pub const STANDARD_PRESSURE_PA: f32 = 101325.0;

/// Standard sea-level temperature in K.
pub const STANDARD_TEMP_K: f32 = 288.15;

/// Standard sea-level density in kg/m^3.
pub const STANDARD_DENSITY_KG_M3: f32 = 1.2250;

/// Specific gas constant for dry air in J/(kg*K).
pub const GAS_CONSTANT_AIR: f32 = 287.05287;

/// Heat capacity ratio for air.
pub const HEAT_CAPACITY_RATIO: f32 = 1.4;

/// Standard speed of sound in m/s.
pub const STANDARD_SPEED_OF_SOUND_MPS: f32 = 340.294;

/// Tropospheric lapse rate in K/m.
pub const TROPO_LAPSE_RATE_K_PER_M: f32 = 0.0065;

/// Barometric pressure exponent: (R * L) / g0.
pub const ISA_PRESSURE_EXPONENT: f32 = 0.190263;

/// Density altitude exponent.
pub const ISA_DENSITY_EXPONENT: f32 = 0.234969;

/// Compute air density from static pressure and temperature.
pub fn air_density(static_p_pa: f32, temp_k: f32) -> Result<f32, AirspeedError> {
    if !static_p_pa.is_finite() || !temp_k.is_finite() {
        return Err(AirspeedError::NonFinite);
    }
    if static_p_pa <= 0.0 {
        return Err(AirspeedError::InvalidPressure);
    }
    if temp_k <= 0.0 {
        return Err(AirspeedError::InvalidTemperature);
    }
    let rho = static_p_pa / (GAS_CONSTANT_AIR * temp_k);
    if !rho.is_finite() || rho <= 0.0 {
        return Err(AirspeedError::InvalidDensity);
    }
    Ok(rho)
}

/// Compute speed of sound in m/s at given temperature.
pub fn speed_of_sound(temp_k: f32) -> Result<f32, AirspeedError> {
    if !temp_k.is_finite() {
        return Err(AirspeedError::NonFinite);
    }
    if temp_k <= 0.0 {
        return Err(AirspeedError::InvalidTemperature);
    }
    let a = libm::sqrtf(HEAT_CAPACITY_RATIO * GAS_CONSTANT_AIR * temp_k);
    Ok(a)
}

/// Incompressible indicated airspeed in m/s.
pub fn incompressible_ias(diff_p_pa: f32) -> Result<f32, AirspeedError> {
    if !diff_p_pa.is_finite() {
        return Err(AirspeedError::NonFinite);
    }
    if diff_p_pa < 0.0 {
        return Err(AirspeedError::NegativeDifferentialPressure);
    }
    let v2 = (2.0 * diff_p_pa) / STANDARD_DENSITY_KG_M3;
    Ok(libm::sqrtf(v2))
}

/// Compute Mach number from impact and static pressure.
pub fn impact_pressure_to_mach(diff_p_pa: f32, static_p_pa: f32) -> Result<f32, AirspeedError> {
    if !diff_p_pa.is_finite() || !static_p_pa.is_finite() {
        return Err(AirspeedError::NonFinite);
    }
    if diff_p_pa < 0.0 {
        return Err(AirspeedError::NegativeDifferentialPressure);
    }
    if static_p_pa <= 0.0 {
        return Err(AirspeedError::InvalidPressure);
    }
    if diff_p_pa == 0.0 {
        return Ok(0.0);
    }
    let pr = diff_p_pa / static_p_pa;
    let base = pr + 1.0;
    let factor = libm::powf(base, 2.0 / 7.0) - 1.0;
    if factor < 0.0 {
        return Ok(0.0);
    }
    let m2 = 5.0 * factor;
    let mach = libm::sqrtf(m2);
    if mach >= 1.0 {
        return Err(AirspeedError::SupersonicFlow);
    }
    Ok(mach)
}

/// Calibrated airspeed (CAS) with subsonic compressibility correction.
pub fn calibrated_airspeed(diff_p_pa: f32) -> Result<f32, AirspeedError> {
    if !diff_p_pa.is_finite() {
        return Err(AirspeedError::NonFinite);
    }
    if diff_p_pa < 0.0 {
        return Err(AirspeedError::NegativeDifferentialPressure);
    }
    if diff_p_pa == 0.0 {
        return Ok(0.0);
    }
    let pr = diff_p_pa / STANDARD_PRESSURE_PA;
    let base = pr + 1.0;
    let factor = libm::powf(base, 2.0 / 7.0) - 1.0;
    if factor < 0.0 {
        return Ok(0.0);
    }
    let cas = STANDARD_SPEED_OF_SOUND_MPS * libm::sqrtf(5.0 * factor);
    Ok(cas)
}

/// Compute true airspeed (TAS) in m/s from pitot-static state.
pub fn true_airspeed(diff_p_pa: f32, static_p_pa: f32, temp_k: f32) -> Result<f32, AirspeedError> {
    let mach = impact_pressure_to_mach(diff_p_pa, static_p_pa)?;
    let a = speed_of_sound(temp_k)?;
    Ok(mach * a)
}

/// Compute equivalent airspeed (EAS) in m/s from TAS and density.
pub fn equivalent_airspeed(tas_mps: f32, density_kg_m3: f32) -> Result<f32, AirspeedError> {
    if !tas_mps.is_finite() || !density_kg_m3.is_finite() {
        return Err(AirspeedError::NonFinite);
    }
    if tas_mps < 0.0 {
        return Err(AirspeedError::NegativeAirspeed);
    }
    if density_kg_m3 <= 0.0 {
        return Err(AirspeedError::InvalidDensity);
    }
    let sigma = density_kg_m3 / STANDARD_DENSITY_KG_M3;
    Ok(tas_mps * libm::sqrtf(sigma))
}

/// Pressure altitude in meters from static pressure.
pub fn pressure_altitude(static_p_pa: f32) -> Result<f32, AirspeedError> {
    if !static_p_pa.is_finite() {
        return Err(AirspeedError::NonFinite);
    }
    if static_p_pa <= 0.0 {
        return Err(AirspeedError::InvalidPressure);
    }
    let pr = static_p_pa / STANDARD_PRESSURE_PA;
    let factor = 1.0 - libm::powf(pr, ISA_PRESSURE_EXPONENT);
    let scale = STANDARD_TEMP_K / TROPO_LAPSE_RATE_K_PER_M;
    Ok(scale * factor)
}

/// Density altitude in meters from static pressure and temperature.
pub fn density_altitude(static_p_pa: f32, temp_k: f32) -> Result<f32, AirspeedError> {
    let rho = air_density(static_p_pa, temp_k)?;
    let dr = rho / STANDARD_DENSITY_KG_M3;
    let factor = 1.0 - libm::powf(dr, ISA_DENSITY_EXPONENT);
    let scale = STANDARD_TEMP_K / TROPO_LAPSE_RATE_K_PER_M;
    Ok(scale * factor)
}

/// Complete pitot-static atmospheric and airspeed telemetry state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirspeedState {
    /// Incompressible indicated airspeed in m/s.
    pub ias_mps: f32,
    /// Calibrated airspeed in m/s.
    pub cas_mps: f32,
    /// Equivalent airspeed in m/s.
    pub eas_mps: f32,
    /// True airspeed in m/s.
    pub tas_mps: f32,
    /// Flight Mach number.
    pub mach: f32,
    /// Local air density in kg/m^3.
    pub density_kg_m3: f32,
    /// Pressure altitude in meters.
    pub pressure_alt_m: f32,
    /// Density altitude in meters.
    pub density_alt_m: f32,
}

/// Pitot-static airspeed estimator with sensor calibration parameters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AirspeedCalculator {
    /// Calibration scale factor for impact differential pressure.
    pub pitot_scale: f32,
    /// Additive static pressure bias correction in Pa.
    pub static_bias_pa: f32,
}

impl AirspeedCalculator {
    /// Create new calculator with default nominal calibration.
    pub const fn new() -> Self {
        Self {
            pitot_scale: 1.0,
            static_bias_pa: 0.0,
        }
    }

    /// Create calculator with custom calibration scale and static bias.
    pub const fn with_calibration(pitot_scale: f32, static_bias_pa: f32) -> Self {
        Self {
            pitot_scale,
            static_bias_pa,
        }
    }

    /// Compute complete airspeed and atmospheric state.
    pub fn compute(
        &self,
        raw_diff_pa: f32,
        raw_static_pa: f32,
        temp_k: f32,
    ) -> Result<AirspeedState, AirspeedError> {
        let cal_diff = raw_diff_pa * self.pitot_scale;
        let cal_static = raw_static_pa + self.static_bias_pa;
