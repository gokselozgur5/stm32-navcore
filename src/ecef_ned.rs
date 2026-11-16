//! Geodetic, ECEF, and local tangent plane NED coordinate transforms.
//!
//! Provides deterministic, zero-allocation transformations using WGS-84.

/// Errors occurring during coordinate conversion or validation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EcefNedError {
    /// Coordinate contains NaN or infinite values.
    NonFinite,
    /// Geodetic latitude outside [-pi/2, pi/2] radians.
    InvalidLatitude,
    /// Geodetic longitude outside [-pi, pi] radians.
    InvalidLongitude,
    /// Position is too close to the Earth's center for inverse projection.
    SingularPoint,
}

impl core::fmt::Display for EcefNedError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NonFinite => write!(f, "coordinate values must be finite"),
            Self::InvalidLatitude => write!(f, "latitude must be within [-pi/2, pi/2]"),
            Self::InvalidLongitude => write!(f, "longitude must be within [-pi, pi]"),
            Self::SingularPoint => write!(f, "position is singular or inside inner core"),
        }
    }
}

/// Geodetic coordinates represented by latitude, longitude, and height.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GeodeticPos {
    /// Geodetic latitude in radians [-pi/2, pi/2].
    pub lat_rad: f64,
    /// Geodetic longitude in radians [-pi, pi].
    pub lon_rad: f64,
    /// Height above reference ellipsoid in meters.
    pub alt_m: f64,
}

impl GeodeticPos {
    /// Construct a validated geodetic position in radians and meters.
    pub fn new(lat_rad: f64, lon_rad: f64, alt_m: f64) -> Result<Self, EcefNedError> {
        if !lat_rad.is_finite() || !lon_rad.is_finite() || !alt_m.is_finite() {
            return Err(EcefNedError::NonFinite);
        }
        let half_pi = core::f64::consts::FRAC_PI_2;
        if lat_rad < -half_pi || lat_rad > half_pi {
            return Err(EcefNedError::InvalidLatitude);
        }
        let pi = core::f64::consts::PI;
        if lon_rad < -pi || lon_rad > pi {
            return Err(EcefNedError::InvalidLongitude);
        }
        Ok(Self { lat_rad, lon_rad, alt_m })
    }

    /// Construct a validated geodetic position from degrees and meters.
    pub fn from_degrees(lat_deg: f64, lon_deg: f64, alt_m: f64) -> Result<Self, EcefNedError> {
        Self::new(lat_deg.to_radians(), lon_deg.to_radians(), alt_m)
    }

    /// Return geodetic latitude converted to degrees.
    pub fn lat_deg(&self) -> f64 {
        self.lat_rad.to_degrees()
    }

    /// Return geodetic longitude converted to degrees.
    pub fn lon_deg(&self) -> f64 {
        self.lon_rad.to_degrees()
    }
}

/// Earth-Centered, Earth-Fixed Cartesian coordinates in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EcefPos {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl EcefPos {
    /// Construct a new ECEF coordinate tuple.
    pub const fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

/// Local Tangent Plane North-East-Down (NED) coordinates in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NedPos {
    pub north: f64,
    pub east: f64,
    pub down: f64,
}

impl NedPos {
    /// Construct a new NED coordinate tuple.
    pub const fn new(north: f64, east: f64, down: f64) -> Self {
        Self { north, east, down }
    }
}
