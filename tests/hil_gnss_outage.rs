use stm32_navcore::ekf::NavEkf;
use stm32_navcore::math::Quaternion;

const DT: f32 = 0.01;
const G: f32 = 9.80665;
const SPEED: f32 = 5.0;

fn level_accel(yaw: f32) -> [f32; 3] {
    let q = Quaternion::new(libm::cosf(yaw * 0.5), 0.0, 0.0, libm::sinf(yaw * 0.5));
    let conj = Quaternion::new(q.w, -q.x, -q.y, -q.z);
    conj.rotate_vec([0.0, 0.0, -G])
}

fn cruise_north(ekf: &mut NavEkf, from: usize, to: usize, gnss: bool) -> f32 {
    let mut worst = 0.0f32;
    for step in from..to {
        ekf.predict(level_accel(0.0), [0.0; 3], DT).unwrap();
        let truth_n = SPEED * (step + 1) as f32 * DT;
        if gnss && step % 20 == 0 {
            let _ = ekf.update_gnss_pos([truth_n, 0.0, 0.0], 0.02, 16.0);
            let _ = ekf.update_gnss_vel([SPEED, 0.0, 0.0], 0.005, 16.0);
        }
        worst = worst.max((ekf.p_ned_m[0] - truth_n).abs());
    }
    worst
}

#[test]
fn outage_error_stays_bounded_and_covariance_grows() {
    let mut ekf = NavEkf::new();
    ekf.v_ned_mps = [SPEED, 0.0, 0.0];
    cruise_north(&mut ekf, 0, 1000, true);
    let before = ekf.cov.0[0][0];
    let worst = cruise_north(&mut ekf, 1000, 2000, false);
    assert!(worst < 1.0, "outage drift {worst}");
    assert!(ekf.cov.0[0][0] > before, "covariance must grow in outage");
}
