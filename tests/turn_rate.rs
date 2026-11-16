use stm32_navcore::turn_rate::*;

#[test]
fn test_turn_rate_error_variants() {
    assert_eq!(
        turn_rate_rad_s(f32::NAN, 0.2, STANDARD_GRAVITY_MPS2),
        Err(TurnRateError::NonFinite)
    );
    assert_eq!(
        turn_rate_rad_s(-10.0, 0.2, STANDARD_GRAVITY_MPS2),
        Err(TurnRateError::InvalidAirspeed)
    );
    assert_eq!(
        turn_rate_rad_s(50.0, 0.2, -9.8),
        Err(TurnRateError::InvalidGravity)
    );
    assert_eq!(
        turn_rate_rad_s(50.0, 1.55, STANDARD_GRAVITY_MPS2),
        Err(TurnRateError::BankAngleOutOfRange)
    );
}

#[test]
fn test_coordinated_turn_rate_known_values() {
    let tas = 50.0;
    let g = STANDARD_GRAVITY_MPS2;
    let bank_rad = 20.0 * DEG_TO_RAD;
    let omega = turn_rate_rad_s(tas, bank_rad, g).unwrap();
    let expected = (g * libm::tanf(bank_rad)) / tas;
    assert!((omega - expected).abs() < 1e-5);

    let deg_rate = turn_rate_deg_s(tas, 20.0, g).unwrap();
    assert!((deg_rate - (expected * RAD_TO_DEG)).abs() < 1e-4);
}

#[test]
fn test_bank_angle_inversion() {
    let tas = 65.0;
    let g = STANDARD_GRAVITY_MPS2;
    let target_rate_deg = 3.0;
    let phi_deg = bank_angle_deg(tas, target_rate_deg, g).unwrap();
    let recovered_rate = turn_rate_deg_s(tas, phi_deg, g).unwrap();
    assert!((recovered_rate - target_rate_deg).abs() < 1e-4);
}

#[test]
fn test_turn_radius_and_centripetal_accel() {
    let tas = 50.0;
    let g = STANDARD_GRAVITY_MPS2;
    let bank_rad = 30.0 * DEG_TO_RAD;
    let r = turn_radius_m(tas, bank_rad, g).unwrap();
    let expected_r = (tas * tas) / (g * libm::tanf(bank_rad));
    assert!((r - expected_r).abs() < 1e-3);

    let a_c = centripetal_accel_mps2(tas, r).unwrap();
    assert!((a_c - (g * libm::tanf(bank_rad))).abs() < 1e-3);
}

#[test]
fn test_load_factor_and_stall_speed() {
    let bank_60 = 60.0 * DEG_TO_RAD;
    let n = load_factor(bank_60).unwrap();
    assert!((n - 2.0).abs() < 1e-3);

    let v_s0 = 30.0;
    let v_s = accelerated_stall_speed(v_s0, bank_60).unwrap();
    let expected_vs = v_s0 * libm::sqrtf(2.0);
    assert!((v_s - expected_vs).abs() < 1e-3);
}

#[test]
fn test_standard_rate_and_time_to_turn() {
    let tas = 100.0;
    let g = STANDARD_GRAVITY_MPS2;
    let phi = standard_rate_bank_angle_rad(tas, g).unwrap();
    let rate = turn_rate_rad_s(tas, phi, g).unwrap();
    assert!((rate - STANDARD_RATE_TURN_RAD_S).abs() < 1e-5);

    let turn_360 = 360.0 * DEG_TO_RAD;
    let t = time_to_turn_s(turn_360, rate).unwrap();
    assert!((t - 120.0).abs() < 0.2);
}
