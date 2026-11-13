use stm32_navcore::geo::{Lla, LtpFrame, Ned};

#[test]
fn test_origin_maps_to_zero_and_roundtrip() {
    let origin = Lla::from_degrees(45.0, 15.0, 200.0).expect("valid coordinates");
    let frame = LtpFrame::from_lla(&origin);

    let ned_origin = frame.ecef_to_ned(&frame.origin_ecef).expect("valid ned");
    assert!(ned_origin.north.abs() < 1e-9);
    assert!(ned_origin.east.abs() < 1e-9);
    assert!(ned_origin.down.abs() < 1e-9);

    let target_ned = Ned::new(123.456, -789.012, 42.0);
    let target_ecef = frame.ned_to_ecef(&target_ned).expect("valid ecef");
    let recovered_ned = frame.ecef_to_ned(&target_ecef).expect("valid ned");
    assert!((recovered_ned.north - target_ned.north).abs() < 1e-9);
    assert!((recovered_ned.east - target_ned.east).abs() < 1e-9);
    assert!((recovered_ned.down - target_ned.down).abs() < 1e-9);
}
