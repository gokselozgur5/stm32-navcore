//! Magnetic declination and inclination WMM2020 lookup table estimator.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MagDeclinationError {
    NonFinite,
    InvalidLatitude,
    InvalidLongitude,
}

/// Estimated magnetic field angular components.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagFieldEstimate {
    /// Magnetic declination in degrees (-180.0 to 180.0, positive East).
    pub declination_deg: f32,
    /// Magnetic inclination (dip) in degrees (-90.0 to 90.0, positive down).
    pub inclination_deg: f32,
}

impl MagFieldEstimate {
    /// Create a new magnetic field estimate from degrees.
    pub const fn new(declination_deg: f32, inclination_deg: f32) -> Self {
        Self {
            declination_deg,
            inclination_deg,
        }
    }

    /// Return declination in radians.
    pub fn declination_rad(&self) -> f32 {
        self.declination_deg * (core::f32::consts::PI / 180.0)
    }

    /// Return inclination in radians.
    pub fn inclination_rad(&self) -> f32 {
        self.inclination_deg * (core::f32::consts::PI / 180.0)
    }
}
