//! Deterministic `no_std`, zero-alloc navigation core for STM32 Cortex-M4F.
//!
//! Modules:
//!
//! - [`ahrs`]: Madgwick AHRS 6-DOF sensor fusion implementation.
//! - [`airspeed`]: Pitot-static indicated airspeed (IAS) and true airspeed (TAS) calculator.
//! - [`attitude`]: Attitude determination algorithms (Madgwick, Mahony, covariance, SLERP).
//! - [`baro_alt`]: Hypsometric barometric altitude estimator.
//! - [`body_rot`]: NED to Body frame rotation matrix operations.
//! - [`butterworth`]: Second-order low-pass IIR Butterworth filter with bilinear transform.
//! - [`ecef_ned`]: Geodetic, ECEF, and local tangent plane NED coordinate transforms.
//! - [`ekf`]: Extended Kalman Filter module for IMU/GNSS navigation.
//! - [`fdir`]: Fault Detection, Isolation and Recovery (FDIR) for dual IMU sensors.
//! - [`filters`]: Discrete filters and signal conditioners.
//! - [`geo`]: Geodesy module: Earth reference models and coordinate transforms.
//! - [`gnss_dop`]: GNSS Dilution of Precision (DOP) metric calculator.
//! - [`gravity`]: Somigliana normal gravity formula and elevation correction.
//! - [`gravity_wgs84`]: Earth gravity spherical harmonic WGS84 normal gravity calculator.
//! - [`gyro_bias`]: Complementary filter gyroscope bias drift estimator and tracker.
//! - [`hypsometric`]: Barometric altitude hypsometric equation and temperature lapse compensator.
//! - [`ins_strapdown`]: Deterministic INS strapdown integration and Coriolis compensator.
//! - [`kinematics`]: Attitude kinematic propagation and coordinate transform matrices.
//! - [`linalg`]: Fixed-size linear algebra on `Mat<R, C>`.
//! - [`madgwick`]: Madgwick AHRS sensor fusion filter for MARG and IMU configurations.
//! - [`mag_declination`]: Magnetic declination and inclination WMM2020 lookup table estimator.
//! - [`mahony_ahrs`]: Mahony AHRS complementary filter with integral gyro bias tracking.
//! - [`math`]: Math module: Matrix algebra, Quaternion, and tilt-compensated heading.
//! - [`nav`]: Inertial and integrated navigation modules.
//! - [`pitot_tas`]: True airspeed (TAS) estimation from pitot differential pressure and IAS.
//! - [`protocol`]: Telemetry framing, CRC-16-CCITT, COBS encoding, and RC ESC protocols.
//! - [`rk4`]: Runge-Kutta 4th-order (RK4) numerical integrator for ODE propagation.
//! - [`rotation`]: Rotation utilities built on the `math` types.
//! - [`sensors`]: Flight sensor drivers and codecs (ICM-42688-P, BMI088, DPS310, IST8310).
//! - [`slerp`]: Quaternion spherical linear and normalized linear interpolation.
//! - [`turn_rate`]: Circular trajectory turn rate and bank angle aerodynamic calculator.
//! - [`ubx`]: Zero-alloc u-blox Binary UBX Protocol Engine.
//! - [`variometer`]: Barometric vertical speed and variometer estimator.
//! - [`vel_damping`]: Dead reckoning velocity damping and GPS velocity fusion.
//! - [`wind_estimator`]: Wind vector triangle estimator from ground speed, TAS, and heading.
//! - [`wmm_declination`]: Magnetic declination polynomial approximation for navigation.
//! - [`zupt`]: Zero-Velocity Update (ZUPT) stance detector for navigation.
//!
//! ## Supported Attitude Implementations
//!
//! The root re-export `MadgwickAhrs` resolves to [`madgwick::MadgwickAhrs`], which is the
//! actively supported implementation supporting both 6-DOF IMU and 9-DOF MARG fusion.
//! The legacy `ahrs::MadgwickAhrs` is a standalone 6-DOF implementation retained for backward
//! compatibility.
//!
//!
//! ## Target Floating-Point Precision
//!
//! While the Cortex-M4F target provides a single-precision (`f32`) hardware FPU,
//! specific geodesy and high-dynamic navigation routines compute in double-precision
//! (`f64`) using software floating point to preserve numerical precision:
//!
//! - [`ins_strapdown`]: Geodetic navigation state, Earth rotation, and Coriolis compensation.
//! - [`nav::dead_reckoning`]: Geodetic position and velocity propagation on ellipsoidal coordinates.
//! - [`geo::vincenty`]: High-precision ellipsoidal geodesic distance and azimuth calculations.
//! - [`sensors::mag_cal::SphereFit`]: Accumulation and normal equations for magnetometer sphere fitting.
//!
//! See `docs/architecture.md` in the repository for the data flow between modules.
#![cfg_attr(not(test), no_std)]

pub mod ahrs;
pub mod attitude;
pub mod body_rot;
pub mod butterworth;
pub mod ekf;
/// Zero-velocity detector over a sliding window of accelerometer and gyro samples.
pub use zupt::{
    DefaultZuptDetector, StancePhase, ZuptConfig, ZuptDetector, ZuptError, ZuptMetrics,
};
pub type StationaryDetector = ZuptDetector<10>;
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
