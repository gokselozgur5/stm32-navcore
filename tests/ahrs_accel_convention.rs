use stm32_navcore::attitude::Mahony;
use stm32_navcore::math::Quaternion;
use stm32_navcore::MahonyAhrs;

#[test]
fn both_mahony_filters_level_on_the_same_ned_sample() {
    let q0 = Quaternion::new(libm::cosf(0.1), libm::sinf(0.1), 0.0, 0.0);
    let accel = [0.0, 0.0, -9.80665];
    let mut a = Mahony::with_attitude(2.0, 0.0, q0).unwrap();
    let mut b = MahonyAhrs::with_attitude(2.0, 0.0, q0).unwrap();
    for _ in 0..6000 {
        a.update([0.0; 3], accel, 0.01).unwrap();
        b.update([0.0; 3], accel, 0.01).unwrap();
    }
    let roll_a = a.euler_angles_zyx().0;
    let roll_b = b.euler_zyx().0;
    assert!(roll_a.abs() < 0.01, "attitude::Mahony roll {roll_a} rad");
    assert!(roll_b.abs() < 0.01, "MahonyAhrs roll {roll_b} rad");
}
