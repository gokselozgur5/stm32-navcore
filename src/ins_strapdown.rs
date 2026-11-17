//! Deterministic INS strapdown integration and Coriolis compensator.

/// Errors occurring during strapdown mechanization and Coriolis compensation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InsStrapdownError {
    /// Non-finite coordinate, rate, or acceleration encountered.
    NonFinite,
    /// Latitude outside valid range [-pi/2, pi/2] radians.
    InvalidLatitude,
    /// Delta time dt is non-positive or non-finite.
    InvalidDeltaTime,
    /// Altitude outside supported range [-10_000 m, 100_000 m].
    InvalidAltitude,
    /// Quaternion norm is zero or degenerate.
    SingularQuaternion,
}

impl core::fmt::Display for InsStrapdownError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NonFinite => write!(f, "non-finite input encountered"),
            Self::InvalidLatitude => write!(f, "latitude must be within [-pi/2, pi/2]"),
            Self::InvalidDeltaTime => write!(f, "delta time must be positive and finite"),
            Self::InvalidAltitude => write!(f, "altitude must be between -10000 and 100000 m"),
            Self::SingularQuaternion => write!(f, "attitude quaternion is singular"),
        }
    }
}

/// WGS-84 semi-major axis in meters.
pub const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 first eccentricity squared.
pub const WGS84_E2: f64 = 0.006_694_379_990_14;

/// Earth rotation angular velocity in rad/s.
pub const EARTH_ROTATION_RATE: f64 = 7.292_115e-5;

/// Somigliana formula equatorial normal gravity in m/s^2.
pub const GAMMA_E: f64 = 9.780_325_335_9;

/// Somigliana formula parameter k.
pub const SOMIGLIANA_K: f64 = 0.001_931_852_652_41;

/// Computes WGS-84 meridian radius of curvature R_M in meters.
pub fn meridian_radius(lat_rad: f64) -> Result<f64, InsStrapdownError> {
    if !lat_rad.is_finite() {
        return Err(InsStrapdownError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(InsStrapdownError::InvalidLatitude);
    }
    let s = libm::sin(lat_rad);
    let denom = 1.0 - WGS84_E2 * s * s;
    let sqrt_denom = libm::sqrt(denom);
    Ok(WGS84_A * (1.0 - WGS84_E2) / (denom * sqrt_denom))
}

/// Computes WGS-84 prime vertical (transverse) radius of curvature R_N in meters.
pub fn transverse_radius(lat_rad: f64) -> Result<f64, InsStrapdownError> {
    if !lat_rad.is_finite() {
        return Err(InsStrapdownError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(InsStrapdownError::InvalidLatitude);
    }
    let s = libm::sin(lat_rad);
    let denom = 1.0 - WGS84_E2 * s * s;
    Ok(WGS84_A / libm::sqrt(denom))
}

/// Computes Earth rotation rate vector in local NED navigation frame.
pub fn earth_rate_ned(lat_rad: f64) -> Result<[f64; 3], InsStrapdownError> {
    if !lat_rad.is_finite() {
        return Err(InsStrapdownError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(InsStrapdownError::InvalidLatitude);
    }
    let c = libm::cos(lat_rad);
    let s = libm::sin(lat_rad);
    Ok([EARTH_ROTATION_RATE * c, 0.0, -EARTH_ROTATION_RATE * s])
}

/// Computes 3D vector cross product u x v.
#[inline]
pub fn cross_product(u: [f64; 3], v: [f64; 3]) -> [f64; 3] {
    [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ]
}

/// Computes craft transport rate vector in local NED navigation frame.
pub fn transport_rate_ned(
    lat_rad: f64,
    alt_m: f64,
    vel_ned: [f64; 3],
) -> Result<[f64; 3], InsStrapdownError> {
    if !alt_m.is_finite()
        || !vel_ned[0].is_finite()
        || !vel_ned[1].is_finite()
        || !vel_ned[2].is_finite()
    {
        return Err(InsStrapdownError::NonFinite);
    }
    if alt_m < -10_000.0 || alt_m > 100_000.0 {
        return Err(InsStrapdownError::InvalidAltitude);
    }
    let r_m = meridian_radius(lat_rad)?;
    let r_n = transverse_radius(lat_rad)?;
    let tan_lat = libm::tan(lat_rad);
    let v_n = vel_ned[0];
    let v_e = vel_ned[1];
    Ok([v_e / (r_n + alt_m), -v_n / (r_m + alt_m), -v_e * tan_lat / (r_n + alt_m)])
}

/// Computes Coriolis and transport acceleration in NED frame.
pub fn coriolis_acceleration(
    lat_rad: f64,
    alt_m: f64,
    vel_ned: [f64; 3],
) -> Result<[f64; 3], InsStrapdownError> {
    let w_ie = earth_rate_ned(lat_rad)?;
    let w_en = transport_rate_ned(lat_rad, alt_m, vel_ned)?;
    let w_sum = [
        2.0 * w_ie[0] + w_en[0],
        2.0 * w_ie[1] + w_en[1],
        2.0 * w_ie[2] + w_en[2],
    ];
    Ok(cross_product(w_sum, vel_ned))
}

/// Computes Somigliana normal gravity with elevation correction.
pub fn somigliana_gravity(lat_rad: f64, alt_m: f64) -> Result<f64, InsStrapdownError> {
    crate::gravity_wgs84::normal_gravity_elevation(lat_rad, alt_m).map_err(|e| match e {
        crate::gravity_wgs84::Wgs84GravityError::NonFinite => InsStrapdownError::NonFinite,
        crate::gravity_wgs84::Wgs84GravityError::InvalidLatitude => InsStrapdownError::InvalidLatitude,
        crate::gravity_wgs84::Wgs84GravityError::InvalidAltitude => InsStrapdownError::InvalidAltitude,
        crate::gravity_wgs84::Wgs84GravityError::SingularRadius => InsStrapdownError::NonFinite,
    })
}

/// Computes gravity vector in NED frame [0, 0, g].
pub fn gravity_ned(lat_rad: f64, alt_m: f64) -> Result<[f64; 3], InsStrapdownError> {
    let g = somigliana_gravity(lat_rad, alt_m)?;
    Ok([0.0, 0.0, g])
}

/// Navigation state for INS strapdown integration in local navigation frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StrapdownState {
    /// Geodetic latitude in radians.
    pub lat_rad: f64,
    /// Geodetic longitude in radians.
    pub lon_rad: f64,
    /// Geodetic altitude above reference ellipsoid in meters.
    pub alt_m: f64,
    /// Velocity in NED frame [v_North, v_East, v_Down] in m/s.
    pub vel_ned: [f64; 3],
    /// Integrated relative displacement in NED frame in meters.
    pub pos_ned: [f64; 3],
    /// Body-to-NED attitude quaternion [w, x, y, z].
    pub quat: [f64; 4],
}

impl StrapdownState {
    /// Creates a new StrapdownState with initial geodetic position.
    pub fn new(lat_rad: f64, lon_rad: f64, alt_m: f64) -> Self {
        Self {
            lat_rad,
            lon_rad,
            alt_m,
            vel_ned: [0.0, 0.0, 0.0],
            pos_ned: [0.0, 0.0, 0.0],
            quat: [1.0, 0.0, 0.0, 0.0],
        }
    }
}
