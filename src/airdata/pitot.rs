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
