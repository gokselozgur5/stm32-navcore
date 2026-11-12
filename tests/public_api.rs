//! Compile-time checks that the crate-root re-exports stay importable.

use stm32_navcore::{
    CalibrationBlock, DualImuManager, ImuId, ImuReading, StationaryDetector, UbxMessage, UbxNavPvt,
    UbxNavRelPosNed, UbxParser,
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
    assert_eq!(size_of::<NavEkf>(), 300);
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
