//! Somigliana normal gravity model with free-air correction.

use crate::geo::GeoError;

/// Somigliana normal gravity at the surface of the WGS-84 ellipsoid.
pub fn normal_gravity_surface(lat_rad: f64) -> Result<f64, GeoError> {
    crate::geo::gravity_wgs84::somigliana_surface(lat_rad).map_err(|e| match e {
        crate::geo::gravity_wgs84::Wgs84GravityError::NonFinite => GeoError::NonFinite,
        crate::geo::gravity_wgs84::Wgs84GravityError::InvalidLatitude => GeoError::InvalidLatitude,
        crate::geo::gravity_wgs84::Wgs84GravityError::InvalidAltitude => GeoError::NonFinite,
        crate::geo::gravity_wgs84::Wgs84GravityError::SingularRadius => GeoError::NonFinite,
    })
}

/// Somigliana normal gravity with free-air height correction.
pub fn normal_gravity(lat_rad: f64, height_m: f64) -> Result<f64, GeoError> {
    crate::geo::gravity_wgs84::normal_gravity_elevation(lat_rad, height_m).map_err(|e| match e {
        crate::geo::gravity_wgs84::Wgs84GravityError::NonFinite => GeoError::NonFinite,
        crate::geo::gravity_wgs84::Wgs84GravityError::InvalidLatitude => GeoError::InvalidLatitude,
        crate::geo::gravity_wgs84::Wgs84GravityError::InvalidAltitude => GeoError::NonFinite,
        crate::geo::gravity_wgs84::Wgs84GravityError::SingularRadius => GeoError::NonFinite,
    })
}
