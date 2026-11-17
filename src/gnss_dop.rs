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

/// Unit line-of-sight vector from receiver to satellite in local ENU frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SatelliteLos {
    /// East component of unit line-of-sight vector.
    pub x: f32,
    /// North component of unit line-of-sight vector.
    pub y: f32,
    /// Up component of unit line-of-sight vector.
    pub z: f32,
}

impl SatelliteLos {
    /// Creates a line-of-sight vector from pre-normalized components.
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Normalizes a 3D vector to a unit line-of-sight vector.
    pub fn from_vector(x: f32, y: f32, z: f32) -> Result<Self, DopError> {
        if !x.is_finite() || !y.is_finite() || !z.is_finite() {
            return Err(DopError::NonFiniteInput);
        }
        let norm_sq = x * x + y * y + z * z;
        if norm_sq <= 1e-12 {
            return Err(DopError::SingularGeometry);
        }
        let inv_norm = 1.0 / libm::sqrtf(norm_sq);
        Ok(Self {
            x: x * inv_norm,
            y: y * inv_norm,
            z: z * inv_norm,
        })
    }
}

impl SatelliteLos {
    /// Computes unit LOS vector from azimuth and elevation in radians.
    ///
    /// Azimuth is clockwise from North; elevation is above horizon.
    pub fn from_azimuth_elevation(azimuth_rad: f32, elevation_rad: f32) -> Result<Self, DopError> {
        if !azimuth_rad.is_finite() || !elevation_rad.is_finite() {
            return Err(DopError::NonFiniteInput);
        }
        let cos_el = libm::cosf(elevation_rad);
        let sin_el = libm::sinf(elevation_rad);
        let sin_az = libm::sinf(azimuth_rad);
        let cos_az = libm::cosf(azimuth_rad);

        Ok(Self {
            x: cos_el * sin_az, // East
            y: cos_el * cos_az, // North
            z: sin_el,          // Up
        })
    }
}

/// Online incremental normal matrix accumulator for GNSS DOP calculation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DopAccumulator {
    /// Normal matrix G^T * G (4x4 symmetric).
    normal_mat: Mat<4, 4>,
    /// Number of accumulated satellite observations.
    count: usize,
}

impl Default for DopAccumulator {
    fn default() -> Self {
        Self::new()
    }
}

impl DopAccumulator {
    /// Creates a fresh accumulator with zeroed normal matrix.
    pub const fn new() -> Self {
        Self {
            normal_mat: Mat::zeros(),
            count: 0,
        }
    }

    /// Resets the accumulator for a new measurement epoch.
    pub fn reset(&mut self) {
        self.normal_mat = Mat::zeros();
        self.count = 0;
    }

    /// Returns the number of accumulated satellites.
    pub const fn count(&self) -> usize {
        self.count
    }
}

impl DopAccumulator {
    /// Accumulates one satellite LOS observation into normal matrix.
    pub fn add_satellite(&mut self, los: &SatelliteLos) -> Result<(), DopError> {
        if !los.x.is_finite() || !los.y.is_finite() || !los.z.is_finite() {
            return Err(DopError::NonFiniteInput);
        }
        let g = [-los.x, -los.y, -los.z, 1.0f32];
        for r in 0..4 {
            for c in 0..4 {
                self.normal_mat.0[r][c] += g[r] * g[c];
            }
        }
        self.count += 1;
        Ok(())
    }
}

impl DopAccumulator {
    /// Accumulates observation from satellite azimuth and elevation angles.
    pub fn add_azimuth_elevation(&mut self, az_rad: f32, el_rad: f32) -> Result<(), DopError> {
        let los = SatelliteLos::from_azimuth_elevation(az_rad, el_rad)?;
        self.add_satellite(&los)
    }
}

impl DopAccumulator {
    /// Inverts accumulated normal matrix using Cholesky SPD decomposition.
    pub fn invert_normal_matrix(&self) -> Result<Mat<4, 4>, DopError> {
        if self.count < 4 {
            return Err(DopError::InsufficientSatellites);
        }
        spd_inverse(&self.normal_mat).map_err(|_| DopError::SingularGeometry)
    }
}

impl DopAccumulator {
    /// Computes dilution of precision metrics from accumulated satellites.
    pub fn compute(&self) -> Result<GnssDop, DopError> {
        let q = self.invert_normal_matrix()?;
        let (ve, vn, vu, vt) = (q.0[0][0], q.0[1][1], q.0[2][2], q.0[3][3]);
        if ve <= 0.0 || vn <= 0.0 || vu <= 0.0 || vt <= 0.0 {
            return Err(DopError::SingularGeometry);
        }
        let (h_sq, p_sq) = (ve + vn, ve + vn + vu);
        Ok(GnssDop {
            gdop: libm::sqrtf(p_sq + vt),
            pdop: libm::sqrtf(p_sq),
            hdop: libm::sqrtf(h_sq),
            vdop: libm::sqrtf(vu),
            tdop: libm::sqrtf(vt),
            ndop: libm::sqrtf(vn),
            edop: libm::sqrtf(ve),
        })
    }
}

/// Computes GNSS DOP metrics from a slice of satellite LOS vectors.
pub fn compute_dop(satellites: &[SatelliteLos]) -> Result<GnssDop, DopError> {
    if satellites.len() < 4 {
        return Err(DopError::InsufficientSatellites);
    }
    let mut acc = DopAccumulator::new();
    for sat in satellites {
        acc.add_satellite(sat)?;
    }
    acc.compute()
}

/// Computes GNSS DOP metrics from satellite (azimuth_rad, elevation_rad) pairs.
pub fn compute_dop_from_az_el(satellites: &[(f32, f32)]) -> Result<GnssDop, DopError> {
    if satellites.len() < 4 {
        return Err(DopError::InsufficientSatellites);
    }
    let mut acc = DopAccumulator::new();
    for &(az, el) in satellites {
        acc.add_azimuth_elevation(az, el)?;
    }
    acc.compute()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_insufficient_satellites_returns_error() {
        let sats = [
            SatelliteLos::new(1.0, 0.0, 0.0),
            SatelliteLos::new(0.0, 1.0, 0.0),
            SatelliteLos::new(0.0, 0.0, 1.0),
        ];
        assert_eq!(compute_dop(&sats), Err(DopError::InsufficientSatellites));
        assert_eq!(compute_dop(&[]), Err(DopError::InsufficientSatellites));
    }

    #[test]
    fn test_non_finite_input_handling() {
        let sats = [
            SatelliteLos::new(f32::NAN, 0.0, 1.0),
            SatelliteLos::new(0.0, 1.0, 0.0),
            SatelliteLos::new(0.0, 0.0, 1.0),
            SatelliteLos::new(1.0, 1.0, 1.0),
        ];
        assert_eq!(compute_dop(&sats), Err(DopError::NonFiniteInput));
        assert_eq!(
            SatelliteLos::from_vector(f32::NAN, 1.0, 2.0),
            Err(DopError::NonFiniteInput)
        );
    }

    #[test]
    fn test_singular_geometry_returns_error() {
        let sats = [
            SatelliteLos::new(1.0, 0.0, 0.0),
            SatelliteLos::new(1.0, 0.0, 0.0),
            SatelliteLos::new(1.0, 0.0, 0.0),
            SatelliteLos::new(1.0, 0.0, 0.0),
        ];
        assert_eq!(compute_dop(&sats), Err(DopError::SingularGeometry));
    }

    #[test]
    fn test_four_satellite_pyramid_dop() {
        let sats = [
            SatelliteLos::new(0.0, 0.0, 1.0),
            SatelliteLos::new(0.0, 0.8660254, 0.5),
            SatelliteLos::new(0.75, -0.4330127, 0.5),
            SatelliteLos::new(-0.75, -0.4330127, 0.5),
        ];
        let dop = compute_dop(&sats).unwrap();
        assert!(dop.is_valid());
        assert!(dop.pdop > 0.0);
        assert!(dop.hdop > 0.0);
        assert!(dop.vdop > 0.0);
        let gdop_sq = dop.gdop * dop.gdop;
        let pdop_sq = dop.pdop * dop.pdop;
        let tdop_sq = dop.tdop * dop.tdop;
        let hdop_sq = dop.hdop * dop.hdop;
        let vdop_sq = dop.vdop * dop.vdop;
        assert!((gdop_sq - (pdop_sq + tdop_sq)).abs() < 1e-3);
        assert!((pdop_sq - (hdop_sq + vdop_sq)).abs() < 1e-3);
    }

    #[test]
    fn test_eight_satellite_constellation_and_accumulator() {
        let az_el = [
            (0.0f32, 0.5f32),
            (0.785f32, 0.8f32),
            (1.57f32, 0.4f32),
            (2.356f32, 0.9f32),
            (3.14f32, 0.6f32),
            (3.926f32, 0.7f32),
            (4.71f32, 0.3f32),
            (5.497f32, 1.1f32),
        ];
        let dop = compute_dop_from_az_el(&az_el).unwrap();
        assert!(dop.is_valid());
        assert!(dop.pdop < 3.0);
    }

    #[test]
    fn test_los_coordinate_frames() {
        let zenith = SatelliteLos::from_azimuth_elevation(0.0, core::f32::consts::FRAC_PI_2).unwrap();
        assert!(zenith.x.abs() < 1e-6);
        assert!(zenith.y.abs() < 1e-6);
        assert!((zenith.z - 1.0).abs() < 1e-6);

        let east = SatelliteLos::from_azimuth_elevation(core::f32::consts::FRAC_PI_2, 0.0).unwrap();
        assert!((east.x - 1.0).abs() < 1e-6);
        assert!(east.y.abs() < 1e-6);
        assert!(east.z.abs() < 1e-6);
    }
}
