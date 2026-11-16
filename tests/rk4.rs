use stm32_navcore::rk4::rk4_step_scalar;

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

use stm32_navcore::rk4::rk4_step;

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
