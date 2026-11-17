//! GNSS Dilution of Precision (DOP) metric calculator.
//!
//! Provides geometric (GDOP), position (PDOP), horizontal (HDOP),
//! vertical (VDOP), and time (TDOP) precision metrics computed
//! from satellite line-of-sight unit vectors or azimuth/elevation.

use crate::linalg::spd::spd_inverse;
use crate::math::Mat;

/// Errors produced during GNSS DOP metric computation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DopError {
    /// Less than 4 satellites available for 3D position and clock solve.
    InsufficientSatellites,
    /// Satellite geometry is singular or ill-conditioned.
    SingularGeometry,
    /// Input vector or coordinates contained non-finite values (NaN/Inf).
    NonFiniteInput,
}

/// Dilution of precision metrics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GnssDop {
    /// Geometric Dilution of Precision (3D position + time).
    pub gdop: f32,
    /// Position Dilution of Precision (3D position).
    pub pdop: f32,
    /// Horizontal Dilution of Precision (East + North).
    pub hdop: f32,
    /// Vertical Dilution of Precision (Up/Down).
    pub vdop: f32,
    /// Time Dilution of Precision (receiver clock bias).
    pub tdop: f32,
    /// North Dilution of Precision.
    pub ndop: f32,
    /// East Dilution of Precision.
    pub edop: f32,
}

impl GnssDop {
    /// Returns true if all metrics are finite and strictly positive.
    pub fn is_valid(&self) -> bool {
        self.gdop.is_finite()
            && self.gdop > 0.0
            && self.pdop.is_finite()
            && self.pdop > 0.0
            && self.hdop.is_finite()
            && self.hdop > 0.0
            && self.vdop.is_finite()
            && self.vdop > 0.0
            && self.tdop.is_finite()
            && self.tdop > 0.0
    }

    /// Returns true if valid and PDOP is within threshold.
    pub fn is_good(&self, max_pdop: f32) -> bool {
        self.is_valid() && self.pdop <= max_pdop
    }
}
