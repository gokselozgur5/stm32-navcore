//! Extended Kalman Filter module for IMU/GNSS navigation.

use crate::math::{Mat, Quaternion};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EkfError {
    InvalidInput,
    SingularInnovation,
    OutlierRejected,
}
