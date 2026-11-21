use stm32_navcore::ahrs::MadgwickAhrs as AhrsMadgwick;
use stm32_navcore::attitude::Mahony as AttMahony;
use stm32_navcore::madgwick::MadgwickAhrs;
use stm32_navcore::mahony_ahrs::MahonyAhrs;

#[test]
fn test_repro_ahrs_drift_and_divergence() {
    // Madgwick default beta drift: ahrs was 0.033 while madgwick was 0.1
    let default_beta_madgwick = MadgwickAhrs::DEFAULT_BETA_IMU;
    let default_beta_ahrs = AhrsMadgwick::DEFAULT_BETA;
    assert_eq!(default_beta_madgwick, default_beta_ahrs);

    // Mahony default kp drift: mahony_ahrs was 1.0 while attitude was 2.0
    let default_kp_mahony = MahonyAhrs::DEFAULT_KP;
    let default_kp_attitude = AttMahony::DEFAULT_KP;
    assert_eq!(default_kp_mahony, default_kp_attitude);
}
