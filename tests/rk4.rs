use stm32_navcore::rk4::{rk4_step_scalar, rk4_step, rk4_integrate_scalar, Rk4Error};

#[test]
fn scalar_exponential_decay() {
    let mut t = 0.0f32;
    let mut y = 1.0f32;
    let dt = 0.1f32;
    for _ in 0..10 {
        y = rk4_step_scalar(t, y, dt, |_t, val| -val).expect("rk4 step");
        t += dt;
    }
    let exact = 1.0f32 / core::f32::consts::E;
    assert!((y - exact).abs() < 1e-4);
}

#[test]
fn harmonic_oscillator_2d() {
    let mut state = [1.0f32, 0.0f32];
    let mut t = 0.0f32;
    let dt = 0.01f32;
    let steps = 628;
    for _ in 0..steps {
        state = rk4_step(t, &state, dt, |_t, y, dydt| {
            dydt[0] = y[1];
            dydt[1] = -y[0];
        })
        .expect("rk4 step 2d");
        t += dt;
    }
    let expected_x = libm::cosf(t);
    let expected_v = -libm::sinf(t);
    assert!((state[0] - expected_x).abs() < 1e-4);
    assert!((state[1] - expected_v).abs() < 1e-4);
}

#[test]
fn multi_step_scalar_integration() {
    let y = rk4_integrate_scalar(0.0, 1.0, 0.1, 10, |_t, val| val).expect("multi step rk4");
    let exact = core::f32::consts::E;
    assert!((y - exact).abs() < 1e-4);
}

#[test]
fn input_validation_and_errors() {
    assert_eq!(
        rk4_step_scalar(f32::NAN, 1.0, 0.1, |_t, y| y),
        Err(Rk4Error::NonFinite)
    );
    assert_eq!(
        rk4_step_scalar(0.0, 1.0, -0.1, |_t, y| y),
        Err(Rk4Error::InvalidTimeStep)
    );
    assert_eq!(
        rk4_integrate_scalar(0.0, 1.0, 0.1, 0, |_t, y| y),
        Err(Rk4Error::ZeroStepCount)
    );
}
