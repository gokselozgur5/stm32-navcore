//! Geodesy module: Earth reference models and coordinate transforms.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GeoError {
    NonFinite,
    InvalidLatitude,
    InvalidLongitude,
}
