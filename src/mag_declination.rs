//! Magnetic declination and inclination WMM2020 lookup table estimator.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MagDeclinationError {
    NonFinite,
    InvalidLatitude,
    InvalidLongitude,
}
