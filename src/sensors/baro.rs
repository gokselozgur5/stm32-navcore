//! Barometer altitude conversion, compensation and filtering.

use crate::math::{pressure_hpa_to_altitude_m, MathError};

/// Sea-level standard pressure in hPa.
pub const ISA_SEA_LEVEL_HPA: f32 = 1013.25;

/// Altitude measurement the EKF can consume.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaroMeasurement {
    /// Altitude above the reference pressure level in meters.
    pub altitude_m: f32,
    /// Measurement variance in square meters.
    pub variance_m2: f32,
}

impl BaroMeasurement {
    /// Builds a measurement from pressure using the ISA model.
    pub fn from_pressure(p_hpa: f32, p0_hpa: f32, variance_m2: f32) -> Result<Self, MathError> {
        let altitude_m = pressure_hpa_to_altitude_m(p_hpa, p0_hpa)?;
        Ok(Self {
            altitude_m,
            variance_m2,
        })
    }
}

/// ISA sea-level temperature in kelvin.
pub const ISA_SEA_LEVEL_K: f32 = 288.15;

/// Error from temperature-compensated altitude.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BaroError {
    Pressure(MathError),
    Temperature,
}

/// Altitude in meters with the ISA height scaled by measured over ISA temperature.
///
/// The reference is the ISA temperature at the altitude being corrected
/// (sea-level temperature minus the 6.5 K/km lapse), not sea-level
/// temperature, so ISA conditions return the ISA altitude unchanged.
pub fn compensated_altitude_m(p_hpa: f32, p0_hpa: f32, temp_c: f32) -> Result<f32, BaroError> {
    let temp_k = temp_c + 273.15;
    if !temp_k.is_finite() || temp_k <= 0.0 {
        return Err(BaroError::Temperature);
    }
    let isa = pressure_hpa_to_altitude_m(p_hpa, p0_hpa).map_err(BaroError::Pressure)?;
    let isa_temp_k = ISA_SEA_LEVEL_K - 0.0065 * isa;
    if isa_temp_k <= 0.0 {
        return Err(BaroError::Temperature);
    }
    Ok(isa * temp_k / isa_temp_k)
}

/// First-order low-pass filter with a jump-based outlier gate.
#[derive(Clone, Copy, Debug)]
pub struct BaroFilter {
    alpha: f32,
    gate_m: f32,
    max_rejects: u8,
    rejects: u8,
    state: Option<f32>,
}

impl BaroFilter {
    /// `alpha` in (0, 1], `gate_m` is the max accepted jump, `max_rejects`
    /// consecutive rejections force acceptance of a sustained jump.
    pub const fn new(alpha: f32, gate_m: f32, max_rejects: u8) -> Self {
        Self {
            alpha,
            gate_m,
            max_rejects,
            rejects: 0,
            state: None,
        }
    }
}

impl BaroFilter {
    /// Feeds one altitude sample; returns the filtered value or None if rejected.
    pub fn update(&mut self, alt_m: f32) -> Option<f32> {
        if !alt_m.is_finite() {
            return None;
        }
        let Some(prev) = self.state else {
            self.state = Some(alt_m);
            return self.state;
        };
        let jump = if alt_m > prev {
            alt_m - prev
        } else {
            prev - alt_m
        };
        if jump > self.gate_m && self.rejects < self.max_rejects {
            self.rejects += 1;
            return None;
        }
        if jump > self.gate_m {
            self.rejects = 0;
            self.state = Some(alt_m);
            return self.state;
        }
        self.rejects = 0;
        let next = prev + self.alpha * (alt_m - prev);
        self.state = Some(next);
        self.state
    }
}
