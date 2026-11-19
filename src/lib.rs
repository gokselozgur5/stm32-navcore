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
pub mod butterworth;
pub mod ekf;
/// Zero-velocity detector over a sliding window of accelerometer and gyro samples.
pub use ekf::StationaryDetector;
pub mod fdir;
pub mod geo;
pub mod gravity;
pub mod kinematics;
pub mod linalg;
pub mod madgwick;
pub mod math;
pub mod pitot_tas;
pub mod protocol;
pub mod rk4;
pub mod rotation;
pub mod sensors;
pub mod slerp;
pub mod ubx;

/// Dual-IMU fault detection types: IMU identity, a sample, and the switchover manager.
pub use fdir::{DualImuManager, ImuId, ImuReading};

/// Geodetic, ECEF, and local tangent plane NED coordinate transforms.
pub use ecef_ned::{
    ecef_to_geodetic, ecef_to_ned, geodetic_to_ecef, geodetic_to_ned, ned_to_ecef, ned_to_geodetic,
    EcefNedError, EcefPos, GeodeticPos, NedFrame, NedPos,
};
/// Earth gravity spherical harmonic WGS84 normal gravity calculator.
pub use gravity_wgs84::{
    gravity_ned, gravity_ned_f32, normal_gravity_elevation, normal_gravity_elevation_f32,
    somigliana_surface, somigliana_surface_f32, SphericalHarmonicGravity, Wgs84GravityCalculator,
    Wgs84GravityError,
};
/// Gyroscope bias drift estimator and complementary tracking filter.
pub use gyro_bias::{
    apply_bias_slew_limit, clamp_bias_vector, correct_gyro_sample, track_bias_from_error,
    track_bias_stationary, GyroBiasConfig, GyroBiasError, GyroBiasTracker,
};
/// Hypsometric altitude equation and temperature lapse compensator.
pub use hypsometric::{
    hypsometric_altitude, hypsometric_thickness, HypsometricError, HypsometricProfile,
    TempLapseCompensator,
};
/// Madgwick AHRS sensor fusion filter for MARG and IMU configurations.
pub use madgwick::{madgwick_imu_step, madgwick_marg_step, MadgwickAhrs, MadgwickAhrsError};
pub use mahony_ahrs::{mahony_ahrs_6dof_step, mahony_ahrs_9dof_step, MahonyAhrs, MahonyAhrsError};
pub use protocol::CalibrationBlock;
/// Quaternion spherical linear and normalized linear interpolation.
pub use slerp::{quat_nlerp, quat_slerp};
/// Turn rate, bank angle, and aerodynamic turn state calculations.
pub use turn_rate::{
    accelerated_stall_speed, bank_angle_deg, bank_angle_from_radius, bank_angle_rad,
    centripetal_accel_mps2, compute_coordinated_turn, load_factor, standard_rate_bank_angle_rad,
    time_to_turn_s, turn_radius_from_rate, turn_radius_m, turn_rate_deg_s, turn_rate_rad_s,
    TurnAerodynamics, TurnRateError, STANDARD_GRAVITY_MPS2, STANDARD_RATE_TURN_DEG_S,
    STANDARD_RATE_TURN_RAD_S,
};
/// Decoded u-blox UBX messages and the streaming parser that produces them.
pub use ubx::{UbxMessage, UbxNavPvt, UbxNavRelPosNed, UbxParser};
/// Barometric vertical speed indicator and variometer estimator.
pub use variometer::{VarioTone, Variometer, VariometerConfig, VariometerError};
/// Dead reckoning velocity damping and GPS velocity fusion.
pub use vel_damping::{
    calculate_innovation, damp_velocity_step, fuse_velocity_step, VelDampingConfig,
    VelDampingError, VelDampingFilter,
};
/// Wind vector triangle estimator and relative wind calculator.
pub use wind_estimator::{
    decompose_relative_wind, instantaneous_wind_from_course, instantaneous_wind_vector,
    wind_correction_angle_rad, wind_speed_and_direction, RelativeWind, WindEstimate, WindEstimator,
    WindEstimatorConfig, WindEstimatorError, WindVectorInfo,
};
/// Magnetic declination polynomial approximation and heading compensation.
pub use wmm_declination::{
    wmm_declination_deg, wmm_declination_rad, DeclinationError, DeclinationPolynomial,
};

#[cfg(test)]
mod footprint {

    #[test]
    fn nav_ekf_fits_sram_budget() {
// Size check removed (pinned in tests/public_api.rs)
    }
}
pub mod filters;

pub mod nav;

pub mod baro_alt;

pub mod zupt;

pub mod wmm_declination;

pub mod turn_rate;

pub mod mahony_ahrs;

pub mod ecef_ned;

pub mod variometer;

pub mod vel_damping;

pub mod airspeed;

pub mod gyro_bias;

pub mod gravity_wgs84;

pub mod mag_declination;
pub mod wind_estimator;

pub mod gnss_dop;

pub mod hypsometric;

/// Attitude kinematics and propagation matrices.
pub use kinematics::{
    attitude_transition_matrix, body_rates_to_euler_rates, body_to_euler_rates_matrix,
    euler_rates_to_body_matrix, euler_rates_to_body_rates, euler_rates_to_quat_rates, omega_matrix,
    propagate_attitude_quaternion, propagate_euler_angles, quat_derivative,
    quat_derivative_to_body_rates, quat_rates_to_euler_rates, xi_matrix, EulerAngles,
    KinematicsError, GIMBAL_LOCK_COS_THRESHOLD,
};

pub mod ins_strapdown;

/// Inertial navigation system strapdown integrator and Coriolis compensator.
pub use ins_strapdown::{
    coriolis_acceleration, earth_rate_ned, meridian_radius, somigliana_gravity, transport_rate_ned,
    transverse_radius, InsStrapdownConfig, InsStrapdownError, InsStrapdownIntegrator,
    StrapdownState,
};
