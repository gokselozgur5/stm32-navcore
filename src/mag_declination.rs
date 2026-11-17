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

impl MagDeclinationLut {
    /// Create a new lookup table with provided grid values.
    pub const fn new(
        declination: [[f32; WMM_LON_COLS]; WMM_LAT_ROWS],
        inclination: [[f32; WMM_LON_COLS]; WMM_LAT_ROWS],
    ) -> Self {
        Self {
            declination,
            inclination,
        }
    }

    /// Normalize longitude in degrees to [-180.0, 180.0].
    pub fn normalize_lon_deg(lon_deg: f32) -> f32 {
        let mut lon = lon_deg;
        while lon > 180.0 {
            lon -= 360.0;
        }
        while lon < -180.0 {
            lon += 360.0;
        }
        lon
    }

    /// Compute cell indices and bilinear interpolation weights.
    fn cell_weights(lat_deg: f32, lon_deg: f32) -> (usize, usize, usize, usize, f32, f32) {
        let lat_clamped = lat_deg.clamp(-90.0, 90.0);
        let lat_idx = ((lat_clamped - WMM_LAT_MIN) / WMM_LAT_STEP) as usize;
        let r0 = lat_idx.min(WMM_LAT_ROWS - 2);
        let r1 = r0 + 1;
        let u = ((lat_clamped - (WMM_LAT_MIN + r0 as f32 * WMM_LAT_STEP)) / WMM_LAT_STEP)
            .clamp(0.0, 1.0);

        let lon_norm = Self::normalize_lon_deg(lon_deg);
        let lon_idx = ((lon_norm - WMM_LON_MIN) / WMM_LON_STEP) as usize;
        let c0 = lon_idx.min(WMM_LON_COLS - 2);
        let c1 = c0 + 1;
        let v =
            ((lon_norm - (WMM_LON_MIN + c0 as f32 * WMM_LON_STEP)) / WMM_LON_STEP).clamp(0.0, 1.0);
        (r0, r1, c0, c1, u, v)
    }

    /// Bilinear interpolation over a 2D scalar grid.
    fn interp_2d(
        grid: &[[f32; WMM_LON_COLS]; WMM_LAT_ROWS],
        r0: usize,
        r1: usize,
        c0: usize,
        c1: usize,
        u: f32,
        v: f32,
    ) -> f32 {
        let p00 = grid[r0][c0];
        let p01 = grid[r0][c1];
        let p10 = grid[r1][c0];
        let p11 = grid[r1][c1];
        (1.0 - u) * ((1.0 - v) * p00 + v * p01) + u * ((1.0 - v) * p10 + v * p11)
    }

    /// Estimate magnetic field angular components using bilinear interpolation.
    pub fn estimate(
        &self,
        lat_deg: f32,
        lon_deg: f32,
    ) -> Result<MagFieldEstimate, MagDeclinationError> {
        if !lat_deg.is_finite() || !lon_deg.is_finite() {
            return Err(MagDeclinationError::NonFinite);
        }
        if !(-90.0..=90.0).contains(&lat_deg) {
            return Err(MagDeclinationError::InvalidLatitude);
        }
        if !(-360.0..=360.0).contains(&lon_deg) {
            return Err(MagDeclinationError::InvalidLongitude);
        }
        let (r0, r1, c0, c1, u, v) = Self::cell_weights(lat_deg, lon_deg);
        let dec = Self::interp_2d(&self.declination, r0, r1, c0, c1, u, v);
        let inc = Self::interp_2d(&self.inclination, r0, r1, c0, c1, u, v);
        Ok(MagFieldEstimate::new(dec, inc))
    }
}
