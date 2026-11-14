//! Air data systems: indicated/calibrated airspeed, density and pitot calculations.

pub mod pitot;

pub use pitot::{differential_pressure_to_ias, AirdataError, RHO0_ISA};
