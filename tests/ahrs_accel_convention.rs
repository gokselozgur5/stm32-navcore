use stm32_navcore::ahrs::MadgwickAhrs as AhrsM;
use stm32_navcore::attitude::{Madgwick as AttM, Mahony};
use stm32_navcore::madgwick::MadgwickAhrs as ModM;
use stm32_navcore::math::Quaternion;
use stm32_navcore::MahonyAhrs;

#[test]
fn all_attitude_filters_level_on_the_same_ned_sample() {
    let q0 = Quaternion::new(libm::cosf(0.1), libm::sinf(0.1), 0.0, 0.0);
    let a = [0.0, 0.0, -9.80665];
    let mut m1 = Mahony::with_attitude(2.0, 0.0, q0).unwrap();
    let mut m2 = MahonyAhrs::with_attitude(2.0, 0.0, q0).unwrap();
    let mut g1 = AttM::new(0.5);
    g1.set_quaternion(q0).unwrap();
    let mut g2 = ModM::with_attitude(0.5, q0).unwrap();
    let mut g3 = AhrsM::with_attitude(0.5, q0).unwrap();
    for _ in 0..6000 {
        m1.update([0.0; 3], a, 0.01).unwrap();
        m2.update([0.0; 3], a, 0.01).unwrap();
        g1.update([0.0; 3], a, 0.01).unwrap();
        g2.update([0.0; 3], a, 0.01).unwrap();
        g3.update([0.0; 3], a, 0.01).unwrap();
    }
    let ok_m = m1.euler_angles_zyx().0.abs() < 0.01 && m2.euler_zyx().0.abs() < 0.01;
    let ok_g = g1.quaternion().to_euler_zyx().0.abs() < 0.01
        && g2.euler_zyx().0.abs() < 0.01
        && g3.euler_zyx().0.abs() < 0.01;
    assert!(ok_m && ok_g);
}
