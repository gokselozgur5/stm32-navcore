//! Inertial and integrated navigation modules.

pub mod dead_reckoning;
pub use dead_reckoning::{dead_reckoning_step, DeadReckoningState, NavError};
