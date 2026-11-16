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
