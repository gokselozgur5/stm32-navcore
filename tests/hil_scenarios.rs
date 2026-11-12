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
