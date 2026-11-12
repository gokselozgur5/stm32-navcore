//! Compile-time checks that the crate-root re-exports stay importable.

use stm32_navcore::{
    CalibrationBlock, DualImuManager, ImuId, ImuReading, StationaryDetector, UbxMessage,
    UbxNavPvt, UbxNavRelPosNed, UbxParser,
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
