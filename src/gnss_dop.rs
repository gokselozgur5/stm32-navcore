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
