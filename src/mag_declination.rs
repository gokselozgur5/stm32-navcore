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

/// Number of latitude rows in the magnetic lookup table.
pub const WMM_LAT_ROWS: usize = 7;
/// Number of longitude columns in the magnetic lookup table.
pub const WMM_LON_COLS: usize = 13;
/// Minimum latitude covered by the table in degrees.
pub const WMM_LAT_MIN: f32 = -90.0;
/// Latitude step in degrees.
pub const WMM_LAT_STEP: f32 = 30.0;
/// Minimum longitude covered by the table in degrees.
pub const WMM_LON_MIN: f32 = -180.0;
/// Longitude step in degrees.
pub const WMM_LON_STEP: f32 = 30.0;

/// World Magnetic Model 2020 lookup table for declination and inclination.
#[derive(Clone, Copy, Debug)]
pub struct MagDeclinationLut {
    /// Declination grid in degrees: [row_lat][col_lon].
    pub declination: [[f32; WMM_LON_COLS]; WMM_LAT_ROWS],
    /// Inclination grid in degrees: [row_lat][col_lon].
    pub inclination: [[f32; WMM_LON_COLS]; WMM_LAT_ROWS],
}
