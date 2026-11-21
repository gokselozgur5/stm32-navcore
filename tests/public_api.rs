//! Compile-time checks that the crate-root re-exports stay importable.

use stm32_navcore::{
    quat_nlerp, quat_slerp, wmm_declination_deg, wmm_declination_rad, CalibrationBlock,
    DualImuManager, GyroBiasConfig, GyroBiasError, GyroBiasTracker, ImuId, ImuReading,
    StationaryDetector, UbxMessage, UbxNavPvt, UbxNavRelPosNed, UbxParser, VarioTone, Variometer,
    VariometerConfig, VariometerError, VelDampingConfig, VelDampingFilter,
};

#[test]
fn crate_root_reexports_are_usable() {
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
    assert_eq!(stm32_navcore::STANDARD_GRAVITY_MPS2, 9.80665);
    let turn_rate = stm32_navcore::turn_rate_deg_s(50.0, 15.0, 9.80665).expect("turn rate");
    assert!((turn_rate - 3.011104).abs() < 1e-4);
    let bank_angle = stm32_navcore::bank_angle_deg(50.0, 3.0, 9.80665).expect("bank angle");
    assert!((bank_angle - 14.947165).abs() < 1e-4);
    let _ = stm32_navcore::MahonyAhrs::default();
    let _ = core::mem::size_of::<Variometer>();
    let _ = core::mem::size_of::<VariometerConfig>();
    let _ = core::mem::size_of::<VariometerError>();
    let _ = core::mem::size_of::<VarioTone>();
    let _ = core::mem::size_of::<VelDampingConfig>();
    let _ = core::mem::size_of::<VelDampingFilter>();
    let _ = core::mem::size_of::<GyroBiasConfig>();
    let _ = core::mem::size_of::<GyroBiasError>();
    let _ = core::mem::size_of::<GyroBiasTracker>();
    let _ = core::mem::size_of::<stm32_navcore::Wgs84GravityCalculator>();
    let _ = core::mem::size_of::<stm32_navcore::Wgs84GravityError>();
    let g_eq = stm32_navcore::somigliana_surface(0.0).expect("somigliana equator");
    assert!((g_eq - 9.780327).abs() < 1e-4);
    let g_elev = stm32_navcore::normal_gravity_elevation(0.0, 0.0).expect("normal gravity");
    assert!((g_elev - 9.780327).abs() < 1e-4);
    let g_ned = stm32_navcore::gravity_ned(0.0, 0.0).expect("gravity ned");
    assert_eq!(g_ned[0], 0.0);
    assert_eq!(g_ned[1], 0.0);
    assert!((g_ned[2] - 9.780327).abs() < 1e-4);
}

use core::mem::{align_of, size_of};
use stm32_navcore::ekf::NavEkf;
use stm32_navcore::math::{Mat, Quaternion};

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
