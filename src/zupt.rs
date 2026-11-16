//! Zero-Velocity Update (ZUPT) stance detector for pedestrian and vehicular navigation.

/// Error conditions in ZUPT detection and configuration.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ZuptError {
    NonFinite,
    InvalidWindowSize,
    InvalidThreshold,
}
