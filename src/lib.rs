//! Deterministic `no_std`, zero-alloc navigation core for STM32 Cortex-M4F.
//!
//! Canonical Domain Namespaces:
//!
//! - [`math`]: Linear algebra (`Mat`), quaternions (`Quaternion`), kinematics, and Runge-Kutta numerical integration.
//! - [`attitude`]: Attitude determination algorithms (Madgwick AHRS, Mahony AHRS, gyro bias tracking, covariance).
//! - [`nav`]: Inertial navigation, Extended Kalman Filter (`NavEkf`), FDIR, ZUPT, turn rate, and velocity damping.
//! - [`airdata`]: Pitot-static indicated/true airspeed, barometric altitude, variometer, and wind triangle estimation.
//! - [`filters`]: Signal conditioning filters (Butterworth, Biquad, Median, Low-pass, Alpha-Beta, Slew rate).
//! - [`geo`]: Geodesy reference ellipsoids, WGS-84 gravity, coordinate transforms (ECEF/NED), and magnetic declination.
//! - [`sensors`]: Hardware sensor codecs and drivers (ICM-42688-P, BMI088, DPS310, IST8310, Allan variance).
//! - [`protocol`]: Telemetry framing (UART COBS, CAN 2.0B), RC ESC protocols, calibration blocks, and UBX binary parser.
//!
//! ## Target Floating-Point Precision
//!
//! While the Cortex-M4F target provides a single-precision (`f32`) hardware FPU,
//! specific geodesy and high-dynamic navigation routines compute in double-precision
//! (`f64`) using software floating point to preserve numerical precision:
//!
//! - [`nav::ins_strapdown`]: Geodetic navigation state, Earth rotation, and Coriolis compensation.
//! - [`nav::dead_reckoning`]: Geodetic position and velocity propagation on ellipsoidal coordinates.
//! - [`geo::vincenty`]: High-precision ellipsoidal geodesic distance and azimuth calculations.
//! - [`sensors::mag_cal::SphereFit`]: Accumulation and normal equations for magnetometer sphere fitting.
//!
//! See `docs/architecture.md` in the repository for the data flow between modules.
#![cfg_attr(not(test), no_std)]

pub mod airdata;
pub mod attitude;
pub mod filters;
pub mod math;
pub mod nav;

#[cfg(feature = "f64-geodesy")]
pub mod geo;

#[cfg(feature = "sensors")]
pub mod sensors;

#[cfg(feature = "protocol")]
pub mod protocol;

// Canonical root re-exports for seamless integration
pub use attitude::{
    madgwick_imu_step, madgwick_marg_step, mahony_ahrs_6dof_step, mahony_ahrs_9dof_step,
    MadgwickAhrs, MadgwickAhrsError, MahonyAhrs, MahonyAhrsError,
};
pub use math::rotation::slerp::{nlerp as quat_nlerp, slerp as quat_slerp};
pub use math::{Mat, Quaternion, Vec3};
pub use nav::{
    DefaultZuptDetector, DualImuManager, ImuId, ImuReading, NavEkf, StationaryDetector, ZuptConfig,
    ZuptDetector,
};

#[cfg(test)]
mod footprint {
    use crate::attitude::MadgwickAhrs;
    use crate::filters::MedianFilter;
    use crate::nav::NavEkf;
    use core::mem::size_of;

    #[test]
    fn nav_ekf_fits_sram_budget() {
        assert_eq!(size_of::<NavEkf>(), 312);
        assert_eq!(size_of::<MadgwickAhrs>(), 20);
        assert_eq!(size_of::<MedianFilter<9>>(), 56);

        assert!(size_of::<NavEkf>() <= 512);
        assert!(size_of::<MadgwickAhrs>() <= 64);
        assert!(size_of::<MedianFilter<9>>() <= 128);

        #[cfg(feature = "sensors")]
        {
            use crate::sensors::allan::AllanEstimator;
            assert_eq!(size_of::<AllanEstimator<128, 5>>(), 1128);
            assert!(size_of::<AllanEstimator<128, 5>>() <= 2048);
        }
    }
}
