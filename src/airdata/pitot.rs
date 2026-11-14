//! Pitot static tube indicated airspeed calculation from differential pressure.

/// Sea-level standard atmospheric air density in kg/m^3 (ISA).
pub const RHO0_ISA: f32 = 1.225;

/// Error conditions encountered in air data calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AirdataError {
    /// Non-finite differential pressure or air density.
    NonFinite,
    /// Invalid non-positive air density.
    InvalidDensity,
}

/// Calculate indicated airspeed from dynamic (differential) pressure.
pub fn differential_pressure_to_ias(dp_pa: f32, rho_kg_m3: f32) -> Result<f32, AirdataError> {
    if !dp_pa.is_finite() || !rho_kg_m3.is_finite() {
        return Err(AirdataError::NonFinite);
    }
    if rho_kg_m3 <= 0.0 {
        return Err(AirdataError::InvalidDensity);
    }
    let dp_clamped = if dp_pa < 0.0 { 0.0 } else { dp_pa };
    Ok(libm::sqrtf(2.0 * dp_clamped / rho_kg_m3))
}
