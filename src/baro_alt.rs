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
pub const TROPOSPHERE_TEMP_SCALE: f32 = STANDARD_SEA_LEVEL_TEMP_K / STANDARD_LAPSE_RATE_K_PER_M;

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
pub fn altitude_from_pressure_qnh(pressure_pa: f32, qnh_pa: f32) -> Result<f32, BaroAltError> {
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

fn validate_baro_inputs(p: f32, p_ref: f32, t_ref: f32, lapse: f32) -> Result<(), BaroAltError> {
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

/// Computes hypsometric altitude given reference conditions and lapse rate.
pub fn hypsometric_altitude(
    pressure_pa: f32,
    p_ref_pa: f32,
    t_ref_k: f32,
    lapse_rate: f32,
) -> Result<f32, BaroAltError> {
    validate_baro_inputs(pressure_pa, p_ref_pa, t_ref_k, lapse_rate)?;
    let ratio = pressure_pa / p_ref_pa;
    if lapse_rate < 1e-7 {
        let scale = (GAS_CONSTANT_DRY_AIR * t_ref_k) / STANDARD_GRAVITY_M_S2;
        return Ok(scale * -libm::logf(ratio));
    }
    let exp = (GAS_CONSTANT_DRY_AIR * lapse_rate) / STANDARD_GRAVITY_M_S2;
    Ok((t_ref_k / lapse_rate) * (1.0 - libm::powf(ratio, exp)))
}

/// Computes atmospheric pressure in Pascals at altitude under standard ISA conditions.
pub fn pressure_at_altitude_isa(altitude_m: f32) -> Result<f32, BaroAltError> {
    if !altitude_m.is_finite() {
        return Err(BaroAltError::NonFinite);
    }
    let temp_ratio = 1.0 - (STANDARD_LAPSE_RATE_K_PER_M * altitude_m) / STANDARD_SEA_LEVEL_TEMP_K;
    if temp_ratio <= 0.0 {
        return Err(BaroAltError::InvalidTemperature);
    }
    let inv_exp = STANDARD_GRAVITY_M_S2 / (GAS_CONSTANT_DRY_AIR * STANDARD_LAPSE_RATE_K_PER_M);
    Ok(STANDARD_SEA_LEVEL_PRESSURE_PA * libm::powf(temp_ratio, inv_exp))
}

/// Calibrates QNH sea-level pressure in Pascals from pressure at a known altitude.
pub fn calibrate_qnh(
    measured_pressure_pa: f32,
    known_altitude_m: f32,
) -> Result<f32, BaroAltError> {
    if !measured_pressure_pa.is_finite() || !known_altitude_m.is_finite() {
        return Err(BaroAltError::NonFinite);
    }
    if measured_pressure_pa <= 0.0 {
        return Err(BaroAltError::InvalidPressure);
    }
    let temp_ratio =
        1.0 - (STANDARD_LAPSE_RATE_K_PER_M * known_altitude_m) / STANDARD_SEA_LEVEL_TEMP_K;
    if temp_ratio <= 0.0 {
        return Err(BaroAltError::InvalidTemperature);
    }
    let inv_exp = STANDARD_GRAVITY_M_S2 / (GAS_CONSTANT_DRY_AIR * STANDARD_LAPSE_RATE_K_PER_M);
    Ok(measured_pressure_pa / libm::powf(temp_ratio, inv_exp))
}

/// Configurable hypsometric barometric altitude estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroAltEstimator {
    /// Reference sea-level pressure in Pascals.
    pub ref_pressure_pa: f32,
    /// Reference sea-level temperature in Kelvin.
    pub ref_temp_k: f32,
    /// Troposphere lapse rate in K/m.
    pub lapse_rate_k_per_m: f32,
}

impl Default for BaroAltEstimator {
    fn default() -> Self {
        Self::standard()
    }
}

impl BaroAltEstimator {
    /// Creates estimator under standard ISA sea-level conditions.
    pub const fn standard() -> Self {
        Self {
            ref_pressure_pa: STANDARD_SEA_LEVEL_PRESSURE_PA,
            ref_temp_k: STANDARD_SEA_LEVEL_TEMP_K,
            lapse_rate_k_per_m: STANDARD_LAPSE_RATE_K_PER_M,
        }
    }

    /// Creates estimator with specified QNH pressure under ISA lapse rate.
    pub fn with_qnh(qnh_pa: f32) -> Result<Self, BaroAltError> {
        if !qnh_pa.is_finite() {
            return Err(BaroAltError::NonFinite);
        }
        if qnh_pa <= 0.0 {
            return Err(BaroAltError::InvalidReferencePressure);
        }
        Ok(Self {
            ref_pressure_pa: qnh_pa,
            ref_temp_k: STANDARD_SEA_LEVEL_TEMP_K,
            lapse_rate_k_per_m: STANDARD_LAPSE_RATE_K_PER_M,
        })
    }

    /// Creates estimator with custom reference conditions.
    pub fn with_custom_reference(
        p_ref_pa: f32,
        t_ref_k: f32,
        lapse_rate: f32,
    ) -> Result<Self, BaroAltError> {
        validate_baro_inputs(p_ref_pa, p_ref_pa, t_ref_k, lapse_rate)?;
        Ok(Self {
            ref_pressure_pa: p_ref_pa,
            ref_temp_k: t_ref_k,
            lapse_rate_k_per_m: lapse_rate,
        })
    }

    /// Estimates altitude from measured pressure using configured parameters.
    pub fn estimate_altitude(&self, pressure_pa: f32) -> Result<f32, BaroAltError> {
        hypsometric_altitude(
            pressure_pa,
            self.ref_pressure_pa,
            self.ref_temp_k,
            self.lapse_rate_k_per_m,
        )
    }

    /// Calibrates reference pressure using pressure measured at known altitude.
    pub fn calibrate_at_known_altitude(
        &mut self,
        measured_pressure_pa: f32,
        known_altitude_m: f32,
    ) -> Result<(), BaroAltError> {
        let qnh = calibrate_qnh(measured_pressure_pa, known_altitude_m)?;
        self.ref_pressure_pa = qnh;
        Ok(())
    }

    /// Computes relative altitude difference between two pressure levels.
    pub fn relative_altitude(
        &self,
        pressure_pa: f32,
        baseline_pressure_pa: f32,
    ) -> Result<f32, BaroAltError> {
        let alt1 = self.estimate_altitude(pressure_pa)?;
        let alt0 = self.estimate_altitude(baseline_pressure_pa)?;
        Ok(alt1 - alt0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_isa_sea_level_altitude() {
        let alt = altitude_from_pressure_isa(STANDARD_SEA_LEVEL_PRESSURE_PA).unwrap();
        assert!(alt.abs() < 1e-3);
    }

    #[test]
    fn test_isa_known_altitude_1000m() {
        let p_1000m = 89874.6;
        let alt = altitude_from_pressure_isa(p_1000m).unwrap();
        assert!((alt - 1000.0).abs() < 1.0);
    }

    #[test]
    fn test_pressure_reversibility() {
        let test_altitudes = [0.0, 500.0, 1500.0, 3000.0];
        for &alt in &test_altitudes {
            let p = pressure_at_altitude_isa(alt).unwrap();
            let recovered_alt = altitude_from_pressure_isa(p).unwrap();
            assert!((recovered_alt - alt).abs() < 1e-2);
        }
    }

    #[test]
    fn test_qnh_altitude_offset() {
        let qnh = 102000.0;
        let alt = altitude_from_pressure_qnh(102000.0, qnh).unwrap();
        assert!(alt.abs() < 1e-3);
    }

    #[test]
    fn test_calibrate_qnh() {
        let p_at_500m = pressure_at_altitude_isa(500.0).unwrap();
        let cal_qnh = calibrate_qnh(p_at_500m, 500.0).unwrap();
        assert!((cal_qnh - STANDARD_SEA_LEVEL_PRESSURE_PA).abs() < 1e-1);
    }

    #[test]
    fn test_estimator_struct_methods() {
        let mut est = BaroAltEstimator::standard();
        let alt = est
            .estimate_altitude(STANDARD_SEA_LEVEL_PRESSURE_PA)
            .unwrap();
        assert!(alt.abs() < 1e-3);
        est.calibrate_at_known_altitude(STANDARD_SEA_LEVEL_PRESSURE_PA, 100.0)
            .unwrap();
        assert!(est.ref_pressure_pa > STANDARD_SEA_LEVEL_PRESSURE_PA);
    }

    #[test]
    fn test_hypsometric_isothermal() {
        let alt = hypsometric_altitude(
            STANDARD_SEA_LEVEL_PRESSURE_PA * 0.5,
            STANDARD_SEA_LEVEL_PRESSURE_PA,
            STANDARD_SEA_LEVEL_TEMP_K,
            0.0,
        )
        .unwrap();
        assert!(alt > 5000.0 && alt < 6000.0);
    }

    #[test]
    fn test_relative_altitude() {
        let est = BaroAltEstimator::standard();
        let p0 = STANDARD_SEA_LEVEL_PRESSURE_PA;
        let p1 = pressure_at_altitude_isa(250.0).unwrap();
        let diff = est.relative_altitude(p1, p0).unwrap();
        assert!((diff - 250.0).abs() < 1e-1);
    }

    #[test]
    fn test_invalid_and_non_finite_inputs() {
        assert_eq!(
            altitude_from_pressure_isa(0.0),
            Err(BaroAltError::InvalidPressure)
        );
        assert_eq!(
            altitude_from_pressure_isa(-100.0),
            Err(BaroAltError::InvalidPressure)
        );
        assert_eq!(
            altitude_from_pressure_isa(f32::NAN),
            Err(BaroAltError::NonFinite)
        );
        assert_eq!(
            altitude_from_pressure_qnh(101325.0, 0.0),
            Err(BaroAltError::InvalidReferencePressure)
        );
        assert_eq!(
            hypsometric_altitude(100000.0, 101325.0, 0.0, 0.0065),
            Err(BaroAltError::InvalidTemperature)
        );
        assert_eq!(
            hypsometric_altitude(100000.0, 101325.0, 288.15, -0.001),
            Err(BaroAltError::InvalidLapseRate)
        );
    }
}
