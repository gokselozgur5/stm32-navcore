//! Deterministic `no_std`, zero-alloc navigation core for STM32 Cortex-M4F.
//!
//! Modules:
//!
//! - [`math`]: fixed-size matrices, SO(3) quaternion, tilt-compensated heading.
//! - [`ekf`]: Joseph-form IMU/GNSS extended Kalman filter.
//! - [`fdir`]: dual-IMU fault detection and switchover.
//! - [`sensors`]: flight sensor drivers and register codecs.
//! - [`protocol`]: COBS, CRC-16-CCITT, CAN 2.0B and UART telemetry framing.
//! - [`ubx`]: streaming u-blox UBX parser.
//!
//! See `docs/architecture.md` in the repository for the data flow between modules.
#![cfg_attr(not(test), no_std)]

pub mod ahrs;
pub mod attitude;
pub mod body_rot;
pub mod butterworth;
pub mod ekf;
/// Zero-velocity detector over a sliding window of accelerometer and gyro samples.
pub use ekf::StationaryDetector;
pub mod fdir;
pub mod geo;
pub mod gravity;
pub mod rk4;
pub mod linalg;
pub mod math;
pub mod pitot_tas;
pub mod protocol;
pub mod rotation;
pub mod sensors;
pub mod slerp;
pub mod ubx;

/// Dual-IMU fault detection types: IMU identity, a sample, and the switchover manager.
pub use fdir::{DualImuManager, ImuId, ImuReading};

/// Versioned, checksummed magnetometer and gyro calibration record.
/// Mahony AHRS filter and step functions for 6-DOF and 9-DOF sensor fusion.
pub use mahony_ahrs::{
    mahony_ahrs_6dof_step, mahony_ahrs_9dof_step, MahonyAhrs, MahonyAhrsError,
};
pub use protocol::CalibrationBlock;
/// Quaternion spherical linear and normalized linear interpolation.
pub use slerp::{quat_nlerp, quat_slerp};
/// Magnetic declination polynomial approximation and heading compensation.
pub use wmm_declination::{
    wmm_declination_deg, wmm_declination_rad, DeclinationError, DeclinationPolynomial,
};
/// Turn rate, bank angle, and aerodynamic turn state calculations.
pub use turn_rate::{
    accelerated_stall_speed, bank_angle_deg, bank_angle_from_radius, bank_angle_rad,
    centripetal_accel_mps2, compute_coordinated_turn, load_factor,
    standard_rate_bank_angle_rad, time_to_turn_s, turn_radius_from_rate, turn_radius_m,
    turn_rate_deg_s, turn_rate_rad_s, TurnAerodynamics, TurnRateError,
    STANDARD_GRAVITY_MPS2, STANDARD_RATE_TURN_DEG_S, STANDARD_RATE_TURN_RAD_S,
};
/// Decoded u-blox UBX messages and the streaming parser that produces them.
pub use ubx::{UbxMessage, UbxNavPvt, UbxNavRelPosNed, UbxParser};

#[cfg(test)]
mod footprint {
    use crate::ekf::NavEkf;

    #[test]
    fn nav_ekf_fits_sram_budget() {
        assert!(core::mem::size_of::<NavEkf>() < 2048);
    }
}
pub mod filters;

pub mod nav;

pub mod baro_alt;

pub mod zupt;

pub mod wmm_declination;

pub mod turn_rate;

pub mod mahony_ahrs;
