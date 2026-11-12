//! Flight sensor drivers and codecs.

pub mod baro_mag;
pub use baro_mag::{Dps310Calib, Dps310Driver, Ist8310Driver, SensorError};
pub mod icm42688;
pub use icm42688::{AafConfig, AccelFsr, GyroFsr, Icm42688, Odr, RawSample, ScaledSample};
