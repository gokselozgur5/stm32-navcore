use core::f32::consts::PI;
use stm32_navcore::ekf::NavEkf;
use stm32_navcore::math::Quaternion;

const DT: f32 = 0.01;
const G: f32 = 9.80665;

fn yaw_quat(yaw: f32) -> Quaternion {
    Quaternion::new(libm::cosf(yaw * 0.5), 0.0, 0.0, libm::sinf(yaw * 0.5))
}

fn ideal_imu(a_ned: [f32; 3], yaw: f32, yaw_rate: f32) -> ([f32; 3], [f32; 3]) {
    let q = yaw_quat(yaw);
    let conj = Quaternion::new(q.w, -q.x, -q.y, -q.z);
    let specific = [a_ned[0], a_ned[1], a_ned[2] - G];
    (conj.rotate_vec(specific), [0.0, 0.0, yaw_rate])
}

fn yaw_error(ekf: &NavEkf, truth: f32) -> f32 {
    let (_, _, y) = ekf.att.to_euler_zyx();
    let mut d = y - truth;
    while d > PI {
        d -= 2.0 * PI;
    }
    while d < -PI {
        d += 2.0 * PI;
    }
    d.abs()
}

#[test]
fn hover_holds_position_and_heading_with_gnss_aiding() {
    let mut ekf = NavEkf::new();
    ekf.p_ned_m = [0.0, 0.0, -10.0];
    for step in 0..2000 {
        let (a, w) = ideal_imu([0.0; 3], 0.0, 0.0);
        ekf.predict(a, w, DT).unwrap();
        if step % 20 == 0 {
            let _ = ekf.update_gnss_pos([0.0, 0.0, -10.0], 0.02, 16.0);
            let _ = ekf.update_gnss_vel([0.0; 3], 0.005, 16.0);
        }
    }
    let err = (ekf.p_ned_m[2] + 10.0).abs() + ekf.p_ned_m[0].abs() + ekf.p_ned_m[1].abs();
    assert!(err < 0.1, "hover position error {err}");
    assert!(yaw_error(&ekf, 0.0) < 0.02);
}

#[test]
fn constant_rate_climb_tracks_altitude_within_bound() {
    let mut ekf = NavEkf::new();
    ekf.v_ned_mps = [0.0, 0.0, -2.0];
    let mut worst = 0.0f32;
    for step in 0..1500 {
        let (a, w) = ideal_imu([0.0; 3], 0.0, 0.0);
        ekf.predict(a, w, DT).unwrap();
        let truth_d = -2.0 * (step + 1) as f32 * DT;
        if step % 20 == 0 {
            let _ = ekf.update_gnss_pos([0.0, 0.0, truth_d], 0.02, 16.0);
            let _ = ekf.update_gnss_vel([0.0, 0.0, -2.0], 0.005, 16.0);
        }
        worst = worst.max((ekf.p_ned_m[2] - truth_d).abs());
    }
    assert!(worst < 0.5, "worst climb altitude error {worst}");
    assert!((ekf.v_ned_mps[2] + 2.0).abs() < 0.1);
}

const TURN_SPEED: f32 = 10.0;
const TURN_RATE: f32 = 0.2;

fn turn_truth(t: f32) -> ([f32; 3], [f32; 3]) {
    let r = TURN_SPEED / TURN_RATE;
    let (s, c) = (libm::sinf(TURN_RATE * t), libm::cosf(TURN_RATE * t));
    (
        [r * s, r * (1.0 - c), 0.0],
        [TURN_SPEED * c, TURN_SPEED * s, 0.0],
    )
}

#[test]
fn coordinated_turn_bounds_position_and_yaw_error() {
    let mut ekf = NavEkf::new();
    ekf.v_ned_mps = [TURN_SPEED, 0.0, 0.0];
    let mut worst = 0.0f32;
    for step in 0..3000 {
        let yaw = TURN_RATE * step as f32 * DT;
        let ac = TURN_SPEED * TURN_RATE;
        let a = [-ac * libm::sinf(yaw), ac * libm::cosf(yaw), 0.0];
        let (ab, wb) = ideal_imu(a, yaw, TURN_RATE);
        ekf.predict(ab, wb, DT).unwrap();
        let (pos, vel) = turn_truth(yaw / TURN_RATE + DT);
        if step % 20 == 0 {
            let _ = ekf.update_gnss_pos(pos, 0.02, 16.0);
            let _ = ekf.update_gnss_vel(vel, 0.005, 16.0);
        }
        worst = worst.max(libm::hypotf(
            ekf.p_ned_m[0] - pos[0],
            ekf.p_ned_m[1] - pos[1],
        ));
    }
    assert!(worst < 1.0, "worst turn position error {worst}");
    assert!(yaw_error(&ekf, TURN_RATE * 30.0) < 0.1);
}
