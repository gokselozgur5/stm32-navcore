//! Magnetic declination polynomial approximation for embedded navigation.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DeclinationError {
    NonFinite,
    InvalidLatitude,
    InvalidLongitude,
    InvalidScale,
}
