use std::sync::mpsc::channel;
use std::time::Duration;
use stm32_navcore::nav::ekf::NavEkf;

#[test]
fn large_finite_yaw_observation_returns() {
    let (tx, rx) = channel();
    std::thread::spawn(move || {
        let mut ekf = NavEkf::new();
        let _ = tx.send(ekf.update_mag_yaw(1.0e9, 0.01, 16.0));
    });
    let done = rx.recv_timeout(Duration::from_millis(500));
    assert!(done.is_ok(), "update_mag_yaw(1.0e9, ..) did not return");
}
