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

#[test]
fn reacquisition_pulls_position_back_and_shrinks_covariance() {
    let mut ekf = NavEkf::new();
    ekf.v_ned_mps = [SPEED, 0.0, 0.0];
    cruise_north(&mut ekf, 0, 1000, true);
    cruise_north(&mut ekf, 1000, 2000, false);
    let outage_cov = ekf.cov.0[0][0];
    ekf.p_ned_m[0] += 3.0;
    let worst = cruise_north(&mut ekf, 2000, 3000, true);
    assert!(worst < 3.5, "worst error after reacquisition {worst}");
    let final_err = (ekf.p_ned_m[0] - SPEED * 30.0).abs();
    assert!(final_err < 0.2, "final error {final_err}");
    assert!(ekf.cov.0[0][0] < outage_cov);
}
