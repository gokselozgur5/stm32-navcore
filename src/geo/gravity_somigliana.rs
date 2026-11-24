//! Somigliana normal gravity formula and elevation correction.

pub use crate::geo::gravity_wgs84::Wgs84GravityError as GravityError;

/// WGS-84 equatorial normal gravity in m/s^2 (f32).
pub const GAMMA_E: f32 = crate::geo::gravity_wgs84::GAMMA_E as f32;

/// WGS-84 polar normal gravity in m/s^2 (f32).
pub const GAMMA_P: f32 = crate::geo::gravity_wgs84::GAMMA_P as f32;

/// Somigliana formula parameter k (f32).
pub const SOMIGLIANA_K: f32 = crate::geo::gravity_wgs84::SOMIGLIANA_K as f32;

/// WGS-84 first eccentricity squared (f32).
pub const WGS84_E2: f32 = crate::geo::WGS84_E2 as f32;

/// WGS-84 semi-major axis in meters (f32).
pub const WGS84_A: f32 = crate::geo::WGS84_A as f32;

/// Somigliana normal gravity at the surface of the WGS-84 ellipsoid.
pub fn somigliana_surface(lat_rad: f32) -> Result<f32, GravityError> {
    crate::geo::gravity_wgs84::somigliana_surface_f32(lat_rad)
}

/// Free-air elevation gravity correction in m/s^2.
pub fn elevation_correction(lat_rad: f32, alt_m: f32) -> Result<f32, GravityError> {
    let g0 = somigliana_surface(lat_rad)?;
    let g = normal_gravity(lat_rad, alt_m)?;
    Ok(g - g0)
}

/// Somigliana normal gravity with elevation correction.
pub fn normal_gravity(lat_rad: f32, alt_m: f32) -> Result<f32, GravityError> {
    crate::geo::gravity_wgs84::normal_gravity_elevation_f32(lat_rad, alt_m)
}

/// Normal gravity vector in NED frame [0, 0, g_down] in m/s^2.
pub fn gravity_vector_ned(lat_rad: f32, alt_m: f32) -> Result<[f32; 3], GravityError> {
    crate::geo::gravity_wgs84::gravity_ned_f32(lat_rad, alt_m)
}

/// Somigliana normal gravity at surface in f64.
pub fn somigliana_surface_f64(lat_rad: f64) -> Result<f64, GravityError> {
    crate::geo::gravity_wgs84::somigliana_surface(lat_rad)
}

/// Somigliana normal gravity with elevation in f64.
pub fn normal_gravity_f64(lat_rad: f64, alt_m: f64) -> Result<f64, GravityError> {
    crate::geo::gravity_wgs84::normal_gravity_elevation(lat_rad, alt_m)
}
