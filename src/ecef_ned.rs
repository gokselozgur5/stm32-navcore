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

/// WGS-84 semi-major axis in meters.
pub const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 reciprocal flattening (1/f).
pub const WGS84_INV_F: f64 = 298.257_223_563;

/// WGS-84 flattening: f = 1 / 298.257223563.
pub const WGS84_F: f64 = 1.0 / WGS84_INV_F;

/// WGS-84 semi-minor axis in meters: b = a * (1 - f).
pub const WGS84_B: f64 = 6_356_752.314_245_179;

/// WGS-84 first eccentricity squared: e^2 = 2f - f^2.
pub const WGS84_E2: f64 = 0.006_694_379_990_141_316_5;

/// Transform geodetic coordinates to ECEF Cartesian coordinates.
pub fn geodetic_to_ecef(geo: &GeodeticPos) -> EcefPos {
    let sin_lat = libm::sin(geo.lat_rad);
    let cos_lat = libm::cos(geo.lat_rad);
    let sin_lon = libm::sin(geo.lon_rad);
    let cos_lon = libm::cos(geo.lon_rad);

    let n = WGS84_A / libm::sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);

    let x = (n + geo.alt_m) * cos_lat * cos_lon;
    let y = (n + geo.alt_m) * cos_lat * sin_lon;
    let z = (n * (1.0 - WGS84_E2) + geo.alt_m) * sin_lat;

    EcefPos::new(x, y, z)
}

/// Transform ECEF Cartesian coordinates to geodetic coordinates via Bowring's method.
pub fn ecef_to_geodetic(ecef: &EcefPos) -> Result<GeodeticPos, EcefNedError> {
    if !ecef.x.is_finite() || !ecef.y.is_finite() || !ecef.z.is_finite() {
        return Err(EcefNedError::NonFinite);
    }
    let p = libm::hypot(ecef.x, ecef.y);
    if p < 1e-6 && libm::fabs(ecef.z) < 1e-6 {
        return Err(EcefNedError::SingularPoint);
    }
    let theta = libm::atan2(ecef.z * WGS84_A, p * WGS84_B);
    let (s_th, c_th) = (libm::sin(theta), libm::cos(theta));
    let ep2 = (WGS84_A * WGS84_A - WGS84_B * WGS84_B) / (WGS84_B * WGS84_B);
    let lat_num = ecef.z + ep2 * WGS84_B * s_th * s_th * s_th;
    let lat_den = p - WGS84_E2 * WGS84_A * c_th * c_th * c_th;
    let lat = libm::atan2(lat_num, lat_den);
    let lon = libm::atan2(ecef.y, ecef.x);
    let (s_lat, c_lat) = (libm::sin(lat), libm::cos(lat));
    let n = WGS84_A / libm::sqrt(1.0 - WGS84_E2 * s_lat * s_lat);
    let alt = if libm::fabs(c_lat) > 1e-4 {
        p / c_lat - n
    } else {
        ecef.z / s_lat - n * (1.0 - WGS84_E2)
    };
    GeodeticPos::new(lat, lon, alt)
}

/// Local tangent plane reference frame caching trigonometric components.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NedFrame {
    pub origin_geo: GeodeticPos,
    pub origin_ecef: EcefPos,
    sin_lat: f64,
    cos_lat: f64,
    sin_lon: f64,
    cos_lon: f64,
}

impl NedFrame {
    /// Initialize local tangent plane frame at a given geodetic origin.
    pub fn from_geodetic(origin: &GeodeticPos) -> Self {
        let origin_ecef = geodetic_to_ecef(origin);
        Self {
            origin_geo: *origin,
            origin_ecef,
            sin_lat: libm::sin(origin.lat_rad),
            cos_lat: libm::cos(origin.lat_rad),
            sin_lon: libm::sin(origin.lon_rad),
            cos_lon: libm::cos(origin.lon_rad),
        }
    }
}

impl NedFrame {
    /// Convert an ECEF point into NED coordinates relative to this frame.
    pub fn ecef_to_ned(&self, point: &EcefPos) -> Result<NedPos, EcefNedError> {
        if !point.x.is_finite() || !point.y.is_finite() || !point.z.is_finite() {
            return Err(EcefNedError::NonFinite);
        }
        let dx = point.x - self.origin_ecef.x;
        let dy = point.y - self.origin_ecef.y;
        let dz = point.z - self.origin_ecef.z;

        let north = -self.sin_lat * self.cos_lon * dx
            - self.sin_lat * self.sin_lon * dy
            + self.cos_lat * dz;
        let east = -self.sin_lon * dx + self.cos_lon * dy;
        let down = -self.cos_lat * self.cos_lon * dx
            - self.cos_lat * self.sin_lon * dy
            - self.sin_lat * dz;

        Ok(NedPos::new(north, east, down))
    }
}

impl NedFrame {
    /// Convert NED coordinates back into ECEF coordinates.
    pub fn ned_to_ecef(&self, ned: &NedPos) -> Result<EcefPos, EcefNedError> {
        if !ned.north.is_finite() || !ned.east.is_finite() || !ned.down.is_finite() {
            return Err(EcefNedError::NonFinite);
        }
        let x = self.origin_ecef.x
            - self.sin_lat * self.cos_lon * ned.north
            - self.sin_lon * ned.east
            - self.cos_lat * self.cos_lon * ned.down;
        let y = self.origin_ecef.y
            - self.sin_lat * self.sin_lon * ned.north
            + self.cos_lon * ned.east
            - self.cos_lat * self.sin_lon * ned.down;
        let z = self.origin_ecef.z + self.cos_lat * ned.north - self.sin_lat * ned.down;

        Ok(EcefPos::new(x, y, z))
    }
}

/// Transform ECEF coordinates to NED coordinates relative to a geodetic reference point.
pub fn ecef_to_ned(point: &EcefPos, ref_geo: &GeodeticPos) -> Result<NedPos, EcefNedError> {
    let frame = NedFrame::from_geodetic(ref_geo);
    frame.ecef_to_ned(point)
}

/// Transform NED coordinates to ECEF coordinates relative to a geodetic reference point.
pub fn ned_to_ecef(ned: &NedPos, ref_geo: &GeodeticPos) -> Result<EcefPos, EcefNedError> {
    let frame = NedFrame::from_geodetic(ref_geo);
    frame.ned_to_ecef(ned)
}

impl NedFrame {
    /// Convert a geodetic point directly into NED coordinates.
    pub fn geodetic_to_ned(&self, point: &GeodeticPos) -> Result<NedPos, EcefNedError> {
        let ecef = geodetic_to_ecef(point);
        self.ecef_to_ned(&ecef)
    }

    /// Convert NED coordinates directly into geodetic coordinates.
    pub fn ned_to_geodetic(&self, ned: &NedPos) -> Result<GeodeticPos, EcefNedError> {
        let ecef = self.ned_to_ecef(ned)?;
        ecef_to_geodetic(&ecef)
    }
}
