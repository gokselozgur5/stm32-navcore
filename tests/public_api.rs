//! Compile-time checks that the canonical domain APIs stay importable.

use stm32_navcore::airdata::{VarioTone, Variometer, VariometerConfig, VariometerError};
use stm32_navcore::attitude::{GyroBiasConfig, GyroBiasError, GyroBiasTracker, MahonyAhrs};
use stm32_navcore::geo::gravity_wgs84::{
    gravity_ned, normal_gravity_elevation, somigliana_surface, Wgs84GravityCalculator,
    Wgs84GravityError,
};
use stm32_navcore::geo::wmm_declination::{wmm_declination_deg, wmm_declination_rad};
use stm32_navcore::math::rotation::slerp::{nlerp as quat_nlerp, slerp as quat_slerp};
use stm32_navcore::nav::turn_rate::{bank_angle_deg, turn_rate_deg_s, STANDARD_GRAVITY_MPS2};
use stm32_navcore::nav::vel_damping::{VelDampingConfig, VelDampingFilter};
use stm32_navcore::nav::{DualImuManager, ImuId, ImuReading, StationaryDetector};
use stm32_navcore::protocol::ubx::{UbxMessage, UbxNavPvt, UbxNavRelPosNed, UbxParser};
use stm32_navcore::protocol::CalibrationBlock;

#[test]
fn canonical_domain_apis_are_usable() {
    let _ = ImuId::Primary;
    let _reading = ImuReading {
        accel_mps2: [0.0; 3],
        gyro_rps: [0.0; 3],
    };
    let _ = UbxParser::new();
    let _ = core::mem::size_of::<DualImuManager>();
    let _ = core::mem::size_of::<StationaryDetector>();
    let _ = core::mem::size_of::<CalibrationBlock>();
    let _ = core::mem::size_of::<UbxMessage>();
    let _ = core::mem::size_of::<UbxNavPvt>();
    let _ = core::mem::size_of::<UbxNavRelPosNed>();
    let q = stm32_navcore::math::Quaternion::IDENTITY;
    let slerp_q = quat_slerp(&q, &q, 0.5).expect("slerp identity");
    assert_eq!(slerp_q, stm32_navcore::math::Quaternion::IDENTITY);
    let nlerp_q = quat_nlerp(&q, &q, 0.5).expect("nlerp identity");
    assert_eq!(nlerp_q, stm32_navcore::math::Quaternion::IDENTITY);
    let _ = wmm_declination_deg(0.0, 0.0);
    let _ = wmm_declination_rad(0.0, 0.0);
    assert_eq!(STANDARD_GRAVITY_MPS2, 9.80665);
    let turn_rate = turn_rate_deg_s(50.0, 15.0, 9.80665).expect("turn rate");
    assert!((turn_rate - 3.011104).abs() < 1e-4);
    let bank_angle = bank_angle_deg(50.0, 3.0, 9.80665).expect("bank angle");
    assert!((bank_angle - 14.947165).abs() < 1e-4);
    let _ = MahonyAhrs::default();
    let _ = core::mem::size_of::<Variometer>();
    let _ = core::mem::size_of::<VariometerConfig>();
    let _ = core::mem::size_of::<VariometerError>();
    let _ = core::mem::size_of::<VarioTone>();
    let _ = core::mem::size_of::<VelDampingConfig>();
    let _ = core::mem::size_of::<VelDampingFilter>();
    let _ = core::mem::size_of::<GyroBiasConfig>();
    let _ = core::mem::size_of::<GyroBiasError>();
    let _ = core::mem::size_of::<GyroBiasTracker>();
    let _ = core::mem::size_of::<Wgs84GravityCalculator>();
    let _ = core::mem::size_of::<Wgs84GravityError>();
    let g_eq = somigliana_surface(0.0).expect("somigliana equator");
    assert!((g_eq - 9.780327).abs() < 1e-4);
    let g_elev = normal_gravity_elevation(0.0, 0.0).expect("normal gravity");
    assert!((g_elev - 9.780327).abs() < 1e-4);
    let g_ned = gravity_ned(0.0, 0.0).expect("gravity ned");
    assert_eq!(g_ned[0], 0.0);
    assert_eq!(g_ned[1], 0.0);
    assert!((g_ned[2] - 9.780327).abs() < 1e-4);
}

use core::mem::{align_of, size_of};
use stm32_navcore::math::{Mat, Quaternion};
use stm32_navcore::nav::ekf::NavEkf;

#[test]
fn math_and_ekf_layout_is_packed_f32() {
    assert_eq!(size_of::<Quaternion>(), 16);
    assert_eq!(align_of::<Quaternion>(), 4);
    assert_eq!(size_of::<Mat<3, 3>>(), 36);
    assert_eq!(align_of::<Mat<3, 3>>(), 4);
    const NAV_EKF_SIZE: usize = 312;
    assert_eq!(
        size_of::<NavEkf>(),
        NAV_EKF_SIZE,
        "NavEkf size budget exceeded"
    );
    assert_eq!(align_of::<NavEkf>(), 4);
}

#[test]
fn calibration_and_telemetry_layout_is_packed_f32() {
    use stm32_navcore::protocol::NavTelemetry;
    assert_eq!(size_of::<CalibrationBlock>(), 76);
    assert_eq!(align_of::<CalibrationBlock>(), 4);
    assert_eq!(size_of::<NavTelemetry>(), 44);
    assert_eq!(align_of::<NavTelemetry>(), 4);
}
