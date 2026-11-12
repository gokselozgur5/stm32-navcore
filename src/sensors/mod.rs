//! Flight sensor drivers and codecs.

pub mod baro_mag;
pub use baro_mag::{
    Dps310Calib, Dps310Driver,
    Ist8310Driver, SensorError,
};
