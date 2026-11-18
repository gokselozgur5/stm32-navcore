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
    pub fn new(p_ref: f32, t_ref: f32, lapse: f32, alt_ref: f32) -> Result<Self, HypsometricError> {
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

impl HypsometricProfile {
    /// Calculate geometric altitude from ambient pressure.
    pub fn altitude_at_pressure(&self, pressure_pa: f32) -> Result<f32, HypsometricError> {
        let delta_h = hypsometric_altitude(
            pressure_pa,
            self.ref_pressure_pa,
            self.ref_temp_k,
            self.lapse_rate_k_per_m,
        )?;
        Ok(self.ref_altitude_m + delta_h)
    }
}

impl HypsometricProfile {
    /// Calculate pressure at geometric altitude under this profile.
    pub fn pressure_at_altitude(&self, altitude_m: f32) -> Result<f32, HypsometricError> {
        if !altitude_m.is_finite() {
            return Err(HypsometricError::NonFinite);
        }
        let delta_h = altitude_m - self.ref_altitude_m;
        if self.lapse_rate_k_per_m < 1e-7 {
            let scale = (GAS_CONSTANT_DRY_AIR * self.ref_temp_k) / STANDARD_GRAVITY_M_S2;
            return Ok(self.ref_pressure_pa * libm::expf(-delta_h / scale));
        }
        let temp_ratio = 1.0 - (self.lapse_rate_k_per_m * delta_h) / self.ref_temp_k;
        if temp_ratio <= 0.0 {
            return Err(HypsometricError::InvalidTemperature);
        }
        let inv_exp = STANDARD_GRAVITY_M_S2 / (GAS_CONSTANT_DRY_AIR * self.lapse_rate_k_per_m);
        Ok(self.ref_pressure_pa * libm::powf(temp_ratio, inv_exp))
    }
}

impl HypsometricProfile {
    /// Calculate expected temperature in Kelvin at geometric altitude.
    pub fn temperature_at_altitude(&self, altitude_m: f32) -> Result<f32, HypsometricError> {
        if !altitude_m.is_finite() {
            return Err(HypsometricError::NonFinite);
        }
        let delta_h = altitude_m - self.ref_altitude_m;
        let t = self.ref_temp_k - self.lapse_rate_k_per_m * delta_h;
        if t <= 0.0 {
            return Err(HypsometricError::InvalidTemperature);
        }
        Ok(t)
    }
}

/// Temperature lapse compensator for true altitude and density altitude corrections.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TempLapseCompensator {
    /// Sea-level reference QNH barometric pressure in Pascals.
    pub ref_qnh_pa: f32,
    /// Station or baseline field elevation in meters.
    pub base_elevation_m: f32,
    /// Estimated or standard temperature lapse rate in K/m.
    pub lapse_rate_k_per_m: f32,
}

impl TempLapseCompensator {
    /// Create new compensator with specified QNH, elevation, and lapse rate.
    pub fn new(
        qnh_pa: f32,
        base_elevation_m: f32,
        lapse_rate_k_per_m: f32,
    ) -> Result<Self, HypsometricError> {
        validate_state(qnh_pa, qnh_pa, STANDARD_SEA_LEVEL_TEMP_K)?;
        if !base_elevation_m.is_finite() {
            return Err(HypsometricError::NonFinite);
        }
        if !lapse_rate_k_per_m.is_finite() || lapse_rate_k_per_m < 0.0 || lapse_rate_k_per_m > 0.05
        {
            return Err(HypsometricError::InvalidLapseRate);
        }
        Ok(Self {
            ref_qnh_pa: qnh_pa,
            base_elevation_m,
            lapse_rate_k_per_m,
        })
    }
}

impl TempLapseCompensator {
    /// Create standard sea-level compensator (1013.25 hPa, 0 m, 0.0065 K/m).
    pub fn standard() -> Self {
        Self {
            ref_qnh_pa: STANDARD_SEA_LEVEL_PRESSURE_PA,
            base_elevation_m: 0.0,
            lapse_rate_k_per_m: STANDARD_LAPSE_RATE_K_PER_M,
        }
    }
}

impl Default for TempLapseCompensator {
    fn default() -> Self {
        Self::standard()
    }
}

impl TempLapseCompensator {
    /// Calculate standard ISA temperature in Kelvin at geopotential altitude.
    pub fn isa_temperature_at_altitude(altitude_m: f32) -> Result<f32, HypsometricError> {
        if !altitude_m.is_finite() {
            return Err(HypsometricError::NonFinite);
        }
        let t = STANDARD_SEA_LEVEL_TEMP_K - STANDARD_LAPSE_RATE_K_PER_M * altitude_m;
        if t <= 0.0 {
            return Err(HypsometricError::InvalidTemperature);
        }
        Ok(t)
    }

    /// Compute delta ISA (deviation of measured temperature from ISA) at altitude.
    pub fn isa_temperature_deviation(
        altitude_m: f32,
        measured_temp_k: f32,
    ) -> Result<f32, HypsometricError> {
        if !measured_temp_k.is_finite() || measured_temp_k <= 0.0 {
            return Err(HypsometricError::InvalidTemperature);
        }
        let isa_t = Self::isa_temperature_at_altitude(altitude_m)?;
        Ok(measured_temp_k - isa_t)
    }
}

impl TempLapseCompensator {
    /// Correct indicated barometric altitude to true altitude using measured OAT.
    pub fn true_altitude_from_indicated(
        &self,
        indicated_alt_m: f32,
        measured_temp_k: f32,
    ) -> Result<f32, HypsometricError> {
        if !indicated_alt_m.is_finite() || !measured_temp_k.is_finite() {
            return Err(HypsometricError::NonFinite);
        }
        if measured_temp_k <= 0.0 {
            return Err(HypsometricError::InvalidTemperature);
        }
        let delta_h = indicated_alt_m - self.base_elevation_m;
        let isa_t = Self::isa_temperature_at_altitude(indicated_alt_m)?;
        let delta_isa = measured_temp_k - isa_t;
        let corrected_layer = delta_h * (1.0 + delta_isa / isa_t);
        Ok(self.base_elevation_m + corrected_layer)
    }
}

impl TempLapseCompensator {
    /// Compute true altitude directly from measured ambient pressure and OAT.
    pub fn compensate_altitude(
        &self,
        pressure_pa: f32,
        measured_temp_k: f32,
    ) -> Result<f32, HypsometricError> {
        validate_state(pressure_pa, self.ref_qnh_pa, measured_temp_k)?;
        let pressure_alt = hypsometric_altitude(
            pressure_pa,
            self.ref_qnh_pa,
            STANDARD_SEA_LEVEL_TEMP_K,
            self.lapse_rate_k_per_m,
        )?;
        self.true_altitude_from_indicated(pressure_alt, measured_temp_k)
    }
}

impl TempLapseCompensator {
    /// Compute density altitude in meters from ambient pressure and OAT.
    pub fn density_altitude(
        &self,
        pressure_pa: f32,
        measured_temp_k: f32,
    ) -> Result<f32, HypsometricError> {
        validate_state(pressure_pa, self.ref_qnh_pa, measured_temp_k)?;
        let pressure_alt = hypsometric_altitude(
            pressure_pa,
            STANDARD_SEA_LEVEL_PRESSURE_PA,
            STANDARD_SEA_LEVEL_TEMP_K,
            self.lapse_rate_k_per_m,
        )?;
        let isa_t = Self::isa_temperature_at_altitude(pressure_alt)?;
        let delta_isa = measured_temp_k - isa_t;
        // Density altitude approximation: 118.8 m per 1 K temperature delta.
        Ok(pressure_alt + 118.8 * delta_isa)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hypsometric_thickness_and_isothermal() {
        let thickness = hypsometric_thickness(
            STANDARD_SEA_LEVEL_PRESSURE_PA,
            89874.6,
            STANDARD_SEA_LEVEL_TEMP_K,
        )
        .unwrap();
        assert!(thickness > 950.0 && thickness < 1050.0);

        let alt_iso = hypsometric_altitude(
            50000.0,
            STANDARD_SEA_LEVEL_PRESSURE_PA,
            STANDARD_SEA_LEVEL_TEMP_K,
            0.0,
        )
        .unwrap();
        assert!(alt_iso > 5000.0 && alt_iso < 6500.0);
    }

    #[test]
    fn test_profile_methods_and_reversibility() {
        let profile = HypsometricProfile::standard_isa();
        let alt0 = profile
            .altitude_at_pressure(STANDARD_SEA_LEVEL_PRESSURE_PA)
            .unwrap();
        assert!(alt0.abs() < 1e-2);

        let p_500m = profile.pressure_at_altitude(500.0).unwrap();
        let alt_rec = profile.altitude_at_pressure(p_500m).unwrap();
        assert!((alt_rec - 500.0).abs() < 1e-1);

        let t_500m = profile.temperature_at_altitude(500.0).unwrap();
        assert!((t_500m - (STANDARD_SEA_LEVEL_TEMP_K - 0.0065 * 500.0)).abs() < 1e-3);
    }

    #[test]
    fn test_compensator_true_altitude_and_density_altitude() {
        let comp = TempLapseCompensator::standard();
        let isa_t = TempLapseCompensator::isa_temperature_at_altitude(1000.0).unwrap();
        assert!((isa_t - 281.65).abs() < 1e-2);

        let dev = TempLapseCompensator::isa_temperature_deviation(1000.0, 291.65).unwrap();
        assert!((dev - 10.0).abs() < 1e-2);

        let true_alt_warm = comp.true_altitude_from_indicated(1000.0, 291.65).unwrap();
        assert!(true_alt_warm > 1000.0);

        let true_alt_cold = comp.true_altitude_from_indicated(1000.0, 271.65).unwrap();
        assert!(true_alt_cold < 1000.0);
    }

    #[test]
    fn test_error_handling_and_bounds() {
        assert_eq!(
            hypsometric_thickness(100000.0, 101000.0, 288.15),
            Err(HypsometricError::InvalidPressure)
        );
        assert_eq!(
            hypsometric_altitude(f32::NAN, 101325.0, 288.15, 0.0065),
            Err(HypsometricError::NonFinite)
        );
        assert_eq!(
            hypsometric_altitude(100000.0, 101325.0, -10.0, 0.0065),
            Err(HypsometricError::InvalidTemperature)
        );
        assert_eq!(
            hypsometric_altitude(100000.0, 101325.0, 288.15, -0.01),
            Err(HypsometricError::InvalidLapseRate)
        );
    }
}
