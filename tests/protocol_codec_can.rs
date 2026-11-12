use stm32_navcore::protocol::{CanFrame20B, NavTelemetry, ProtocolError};

#[test]
fn frame_limits() {
    assert!(CanFrame20B::new(CanFrame20B::MAX_EXT_ID, 8, [0; 8]).is_ok());
    assert_eq!(
        CanFrame20B::new(CanFrame20B::MAX_EXT_ID + 1, 0, [0; 8]),
        Err(ProtocolError::InvalidCanId)
    );
    assert!(CanFrame20B::new(0, 0, [0; 8]).is_ok());
    assert_eq!(CanFrame20B::new(0, 9, [0; 8]), Err(ProtocolError::InvalidCanDlc));
}
