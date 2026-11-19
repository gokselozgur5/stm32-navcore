use stm32_navcore::ekf::NavEkf;

#[test]
fn position_integrates_velocity_20_km_from_origin() {
    let mut ekf = NavEkf::new();
    ekf.p_ned_m = [20_000.0, 0.0, 0.0];
    ekf.v_ned_mps = [20.0, 0.0, 0.0];
    for _ in 0..10_000 {
        ekf.predict([0.0, 0.0, -9.80665], [0.0; 3], 0.001).unwrap();
    }
    let moved = ekf.p_ned_m[0] as f64 - 20_000.0;
    assert!(
        (moved - 200.0).abs() < 0.5,
        "moved {moved} m in 10 s at 20 m/s"
    );
}
