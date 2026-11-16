//! Circular trajectory turn rate and bank angle aerodynamic calculator.

/// Errors in turn rate and aerodynamic calculations.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TurnRateError {
    /// Input value is NaN or infinite.
    NonFinite,
    /// Airspeed is non-positive or negative.
    InvalidAirspeed,
    /// Turn radius is non-positive.
    InvalidRadius,
    /// Gravity acceleration is non-positive.
    InvalidGravity,
    /// Bank angle exceeds safe aerodynamic limits.
    BankAngleOutOfRange,
    /// Turn rate is zero when calculating radius.
    ZeroTurnRate,
    /// Stall speed is non-positive.
    InvalidStallSpeed,
}

/// Standard sea-level gravity acceleration in m/s^2.
pub const STANDARD_GRAVITY_MPS2: f32 = 9.80665;

/// Standard rate turn for civil aviation in rad/s (3 deg/s).
pub const STANDARD_RATE_TURN_RAD_S: f32 = 0.05235987756;

/// Standard rate turn in degrees per second (3 deg/s).
pub const STANDARD_RATE_TURN_DEG_S: f32 = 3.0;

/// Maximum allowable bank angle in radians (85 degrees).
pub const MAX_BANK_ANGLE_RAD: f32 = 1.4835298642;

/// Degrees to radians factor.
pub const DEG_TO_RAD: f32 = 0.01745329252;

/// Radians to degrees factor.
pub const RAD_TO_DEG: f32 = 57.295779513;

fn validate_finite(val: f32) -> Result<(), TurnRateError> {
    if !val.is_finite() {
        return Err(TurnRateError::NonFinite);
    }
    Ok(())
}

fn validate_speed_gravity(speed: f32, gravity: f32) -> Result<(), TurnRateError> {
    validate_finite(speed)?;
    validate_finite(gravity)?;
    if speed <= 0.0 {
        return Err(TurnRateError::InvalidAirspeed);
    }
    if gravity <= 0.0 {
        return Err(TurnRateError::InvalidGravity);
    }
    Ok(())
}

fn validate_bank_angle(bank_rad: f32) -> Result<(), TurnRateError> {
    validate_finite(bank_rad)?;
    if libm::fabsf(bank_rad) > MAX_BANK_ANGLE_RAD {
        return Err(TurnRateError::BankAngleOutOfRange);
    }
    Ok(())
}

/// Compute coordinated turn rate in rad/s: omega = (g * tan(phi)) / v.
pub fn turn_rate_rad_s(
    true_airspeed_mps: f32,
    bank_angle_rad: f32,
    gravity_mps2: f32,
) -> Result<f32, TurnRateError> {
    validate_speed_gravity(true_airspeed_mps, gravity_mps2)?;
    validate_bank_angle(bank_angle_rad)?;
    let tan_phi = libm::tanf(bank_angle_rad);
    Ok((gravity_mps2 * tan_phi) / true_airspeed_mps)
}

/// Compute coordinated turn rate in deg/s from bank angle in degrees.
pub fn turn_rate_deg_s(
    true_airspeed_mps: f32,
    bank_angle_deg: f32,
    gravity_mps2: f32,
) -> Result<f32, TurnRateError> {
    validate_finite(bank_angle_deg)?;
    let bank_rad = bank_angle_deg * DEG_TO_RAD;
    let rate_rad = turn_rate_rad_s(true_airspeed_mps, bank_rad, gravity_mps2)?;
    Ok(rate_rad * RAD_TO_DEG)
}

/// Compute bank angle in radians for a given turn rate: phi = atan2(v * omega, g).
pub fn bank_angle_rad(
    true_airspeed_mps: f32,
    turn_rate_rad_s: f32,
    gravity_mps2: f32,
) -> Result<f32, TurnRateError> {
    validate_speed_gravity(true_airspeed_mps, gravity_mps2)?;
    validate_finite(turn_rate_rad_s)?;
    let phi = libm::atan2f(true_airspeed_mps * turn_rate_rad_s, gravity_mps2);
    validate_bank_angle(phi)?;
    Ok(phi)
}

/// Compute bank angle in degrees from turn rate in degrees per second.
pub fn bank_angle_deg(
    true_airspeed_mps: f32,
    turn_rate_deg_s: f32,
    gravity_mps2: f32,
) -> Result<f32, TurnRateError> {
    validate_finite(turn_rate_deg_s)?;
    let rate_rad = turn_rate_deg_s * DEG_TO_RAD;
    let phi_rad = bank_angle_rad(true_airspeed_mps, rate_rad, gravity_mps2)?;
    Ok(phi_rad * RAD_TO_DEG)
}

/// Compute turn radius in meters: R = v^2 / (g * tan(|phi|)).
pub fn turn_radius_m(
    true_airspeed_mps: f32,
    bank_angle_rad: f32,
    gravity_mps2: f32,
) -> Result<f32, TurnRateError> {
    validate_speed_gravity(true_airspeed_mps, gravity_mps2)?;
    validate_bank_angle(bank_angle_rad)?;
    let abs_tan = libm::fabsf(libm::tanf(bank_angle_rad));
    if abs_tan < 1e-6 {
        return Err(TurnRateError::ZeroTurnRate);
    }
    Ok((true_airspeed_mps * true_airspeed_mps) / (gravity_mps2 * abs_tan))
}

/// Compute turn radius in meters from turn rate: R = v / |omega|.
pub fn turn_radius_from_rate(
    true_airspeed_mps: f32,
    turn_rate_rad_s: f32,
) -> Result<f32, TurnRateError> {
    validate_finite(true_airspeed_mps)?;
    validate_finite(turn_rate_rad_s)?;
    if true_airspeed_mps <= 0.0 {
        return Err(TurnRateError::InvalidAirspeed);
    }
    let abs_omega = libm::fabsf(turn_rate_rad_s);
    if abs_omega < 1e-6 {
        return Err(TurnRateError::ZeroTurnRate);
    }
    Ok(true_airspeed_mps / abs_omega)
}

/// Compute required bank angle in radians for a given radius: phi = atan2(v^2, g * R).
pub fn bank_angle_from_radius(
    true_airspeed_mps: f32,
    radius_m: f32,
    gravity_mps2: f32,
) -> Result<f32, TurnRateError> {
    validate_speed_gravity(true_airspeed_mps, gravity_mps2)?;
    validate_finite(radius_m)?;
    if radius_m <= 0.0 {
        return Err(TurnRateError::InvalidRadius);
    }
    let v2 = true_airspeed_mps * true_airspeed_mps;
    let phi = libm::atan2f(v2, gravity_mps2 * radius_m);
    validate_bank_angle(phi)?;
    Ok(phi)
}

/// Compute centripetal acceleration in m/s^2: a_c = v^2 / R.
pub fn centripetal_accel_mps2(
    true_airspeed_mps: f32,
    radius_m: f32,
) -> Result<f32, TurnRateError> {
    validate_finite(true_airspeed_mps)?;
    validate_finite(radius_m)?;
    if true_airspeed_mps < 0.0 {
        return Err(TurnRateError::InvalidAirspeed);
    }
    if radius_m <= 0.0 {
        return Err(TurnRateError::InvalidRadius);
    }
    Ok((true_airspeed_mps * true_airspeed_mps) / radius_m)
}

/// Compute load factor (g-load) in a coordinated turn: n = 1 / cos(phi).
pub fn load_factor(bank_angle_rad: f32) -> Result<f32, TurnRateError> {
    validate_bank_angle(bank_angle_rad)?;
    let cos_phi = libm::cosf(bank_angle_rad);
    if cos_phi <= 0.0 {
        return Err(TurnRateError::BankAngleOutOfRange);
    }
    Ok(1.0 / cos_phi)
}

/// Compute accelerated stall speed: V_s = V_s0 * sqrt(n).
pub fn accelerated_stall_speed(
    v_stall_level_mps: f32,
    bank_angle_rad: f32,
) -> Result<f32, TurnRateError> {
    validate_finite(v_stall_level_mps)?;
    if v_stall_level_mps <= 0.0 {
        return Err(TurnRateError::InvalidStallSpeed);
    }
    let n = load_factor(bank_angle_rad)?;
    Ok(v_stall_level_mps * libm::sqrtf(n))
}

/// Compute bank angle in radians for a standard rate turn (3 deg/s).
pub fn standard_rate_bank_angle_rad(
    true_airspeed_mps: f32,
    gravity_mps2: f32,
) -> Result<f32, TurnRateError> {
    bank_angle_rad(true_airspeed_mps, STANDARD_RATE_TURN_RAD_S, gravity_mps2)
}

/// Compute time in seconds to turn through a heading change in radians.
pub fn time_to_turn_s(turn_angle_rad: f32, turn_rate_rad_s: f32) -> Result<f32, TurnRateError> {
    validate_finite(turn_angle_rad)?;
    validate_finite(turn_rate_rad_s)?;
    let abs_rate = libm::fabsf(turn_rate_rad_s);
    if abs_rate < 1e-6 {
        return Err(TurnRateError::ZeroTurnRate);
    }
    Ok(libm::fabsf(turn_angle_rad) / abs_rate)
}
