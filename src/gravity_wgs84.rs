//! Earth gravity spherical harmonic WGS84 normal gravity calculator.

/// Errors occurring during WGS84 gravity evaluation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wgs84GravityError {
    /// Non-finite coordinate or parameter encountered.
    NonFinite,
    /// Latitude outside valid range [-pi/2, pi/2] radians.
    InvalidLatitude,
    /// Altitude outside supported range [-10_000 m, 100_000 m].
    InvalidAltitude,
    /// Radius too small or singular (r < 1.0 m).
    SingularRadius,
}

/// WGS-84 semi-major axis in meters (a).
pub const WGS84_A: f64 = 6_378_137.0;

/// WGS-84 semi-minor axis in meters (b).
pub const WGS84_B: f64 = 6_356_752.314_245;

/// WGS-84 flattening (f = (a - b) / a).
pub const WGS84_F: f64 = 1.0 / 298.257_223_563;

/// WGS-84 first eccentricity squared (e^2).
pub const WGS84_E2: f64 = 0.006_694_379_990_14;

/// WGS-84 Earth gravitational constant (GM) in m^3/s^2.
pub const WGS84_GM: f64 = 3.986_004_418e14;

/// WGS-84 Earth angular velocity in rad/s.
pub const WGS84_OMEGA: f64 = 7.292_115e-5;

/// WGS-84 second zonal harmonic coefficient J2.
pub const WGS84_J2: f64 = 1.082_629_821_31e-3;

/// WGS-84 fourth zonal harmonic coefficient J4.
pub const WGS84_J4: f64 = -2.370_912_22e-6;

/// WGS-84 equatorial normal gravity in m/s^2 (gamma_e).
pub const GAMMA_E: f64 = 9.780_325_335_9;

/// WGS-84 polar normal gravity in m/s^2 (gamma_p).
pub const GAMMA_P: f64 = 9.832_184_937_8;

/// Somigliana formula parameter k.
pub const SOMIGLIANA_K: f64 = 0.001_931_852_652_41;

/// Somigliana normal gravity at the surface of the WGS-84 ellipsoid in m/s^2.
pub fn somigliana_surface(lat_rad: f64) -> Result<f64, Wgs84GravityError> {
    if !lat_rad.is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(Wgs84GravityError::InvalidLatitude);
    }
    let s = libm::sin(lat_rad);
    let s2 = s * s;
    let num = 1.0 + SOMIGLIANA_K * s2;
    let den = libm::sqrt(1.0 - WGS84_E2 * s2);
    Ok(GAMMA_E * (num / den))
}

/// Somigliana normal gravity at surface in f32 for Cortex-M4F navigation.
pub fn somigliana_surface_f32(lat_rad: f32) -> Result<f32, Wgs84GravityError> {
    if !lat_rad.is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    let half_pi = core::f32::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(Wgs84GravityError::InvalidLatitude);
    }
    let s = libm::sinf(lat_rad);
    let s2 = s * s;
    let num = 1.0f32 + (SOMIGLIANA_K as f32) * s2;
    let den = libm::sqrtf(1.0f32 - (WGS84_E2 as f32) * s2);
    Ok((GAMMA_E as f32) * (num / den))
}

/// Somigliana normal gravity with elevation correction (f64).
pub fn normal_gravity_elevation(lat_rad: f64, alt_m: f64) -> Result<f64, Wgs84GravityError> {
    if !alt_m.is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    if alt_m < -10_000.0 || alt_m > 100_000.0 {
        return Err(Wgs84GravityError::InvalidAltitude);
    }
    let g0 = somigliana_surface(lat_rad)?;
    let m = 0.003_449_786_506_84;
    let s = libm::sin(lat_rad);
    let s2 = s * s;
    let term1 = (2.0 / WGS84_A) * (1.0 + WGS84_F + m - 2.0 * WGS84_F * s2) * alt_m;
    let term2 = (3.0 / (WGS84_A * WGS84_A)) * alt_m * alt_m;
    Ok(g0 * (1.0 - term1 + term2))
}

/// Somigliana normal gravity with elevation correction (f32).
pub fn normal_gravity_elevation_f32(lat_rad: f32, alt_m: f32) -> Result<f32, Wgs84GravityError> {
    if !alt_m.is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    if alt_m < -10_000.0 || alt_m > 100_000.0 {
        return Err(Wgs84GravityError::InvalidAltitude);
    }
    let g0 = somigliana_surface_f32(lat_rad)?;
    let a = WGS84_A as f32;
    let f = WGS84_F as f32;
    let m = 0.003_449_786_5f32;
    let s = libm::sinf(lat_rad);
    let s2 = s * s;
    let term1 = (2.0f32 / a) * (1.0f32 + f + m - 2.0f32 * f * s2) * alt_m;
    let term2 = (3.0f32 / (a * a)) * alt_m * alt_m;
    Ok(g0 * (1.0f32 - term1 + term2))
}

/// Spherical harmonic gravitational and centrifugal acceleration components.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SphericalHarmonicGravity {
    /// Inward radial gravitational acceleration component in m/s^2.
    pub radial: f64,
    /// Northward latitudinal gravitational acceleration component in m/s^2.
    pub north: f64,
    /// Total magnitude of gravitational acceleration vector in m/s^2.
    pub magnitude: f64,
}

/// Gravitational acceleration from J2 and J4 zonal spherical harmonics.
pub fn spherical_harmonic_gravity(
    r_m: f64,
    lat_rad: f64,
) -> Result<SphericalHarmonicGravity, Wgs84GravityError> {
    if !r_m.is_finite() || !lat_rad.is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    if r_m < 1.0 {
        return Err(Wgs84GravityError::SingularRadius);
    }
    let half_pi = core::f64::consts::FRAC_PI_2;
    if lat_rad < -half_pi || lat_rad > half_pi {
        return Err(Wgs84GravityError::InvalidLatitude);
    }
    let (s, c) = (libm::sin(lat_rad), libm::cos(lat_rad));
    let (s2, a_r) = (s * s, WGS84_A / r_m);
    let (a_r2, a_r4) = (a_r * a_r, a_r * a_r * a_r * a_r);
    let p2 = 0.5 * (3.0 * s2 - 1.0);
    let p4 = 0.125 * (35.0 * s2 * s2 - 30.0 * s2 + 3.0);
    let dp2 = 3.0 * s * c;
    let dp4 = 2.5 * (7.0 * s2 - 3.0) * s * c;
    let gm_r2 = WGS84_GM / (r_m * r_m);
    let radial = gm_r2 * (1.0 - 3.0 * WGS84_J2 * a_r2 * p2 - 5.0 * WGS84_J4 * a_r4 * p4);
    let north = -gm_r2 * (WGS84_J2 * a_r2 * dp2 + WGS84_J4 * a_r4 * dp4);
    let mag = libm::sqrt(radial * radial + north * north);
    Ok(SphericalHarmonicGravity { radial, north, magnitude: mag })
}

/// Centrifugal acceleration in ECEF frame from Earth rotation (f64).
pub fn centrifugal_accel_ecef(pos_ecef: [f64; 3]) -> Result<[f64; 3], Wgs84GravityError> {
    if !pos_ecef[0].is_finite() || !pos_ecef[1].is_finite() || !pos_ecef[2].is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    let omega2 = WGS84_OMEGA * WGS84_OMEGA;
    Ok([omega2 * pos_ecef[0], omega2 * pos_ecef[1], 0.0])
}

/// Centrifugal acceleration in ECEF frame (f32).
pub fn centrifugal_accel_ecef_f32(pos_ecef: [f32; 3]) -> Result<[f32; 3], Wgs84GravityError> {
    if !pos_ecef[0].is_finite() || !pos_ecef[1].is_finite() || !pos_ecef[2].is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    let omega = WGS84_OMEGA as f32;
    let omega2 = omega * omega;
    Ok([omega2 * pos_ecef[0], omega2 * pos_ecef[1], 0.0f32])
}

/// Pure gravitational acceleration in ECEF coordinates including J2 perturbation.
pub fn gravitational_accel_ecef_j2(pos: [f64; 3]) -> Result<[f64; 3], Wgs84GravityError> {
    let (x, y, z) = (pos[0], pos[1], pos[2]);
    if !x.is_finite() || !y.is_finite() || !z.is_finite() {
        return Err(Wgs84GravityError::NonFinite);
    }
    let r2 = x * x + y * y + z * z;
    if r2 < 1.0 {
        return Err(Wgs84GravityError::SingularRadius);
    }
    let r = libm::sqrt(r2);
    let r3 = r * r2;
    let a_r2 = (WGS84_A * WGS84_A) / r2;
    let z2_r2 = (z * z) / r2;
    let j2_fac = 1.5 * WGS84_J2 * a_r2;
    let fac_xy = 1.0 - j2_fac * (5.0 * z2_r2 - 1.0);
    let fac_z = 1.0 - j2_fac * (5.0 * z2_r2 - 3.0);
    let k = -WGS84_GM / r3;
    Ok([k * x * fac_xy, k * y * fac_xy, k * z * fac_z])
}
