//! Runge-Kutta 4th-order (RK4) numerical integrator for ODE propagation.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Rk4Error {
    NonFinite,
    InvalidTimeStep,
    ZeroStepCount,
}

#[inline]
pub(crate) fn validate_step(t: f32, dt: f32) -> Result<(), Rk4Error> {
    if !t.is_finite() || !dt.is_finite() {
        return Err(Rk4Error::NonFinite);
    }
    if dt <= 0.0 {
        return Err(Rk4Error::InvalidTimeStep);
    }
    Ok(())
}

/// Single RK4 step for scalar ODE: dy/dt = f(t, y).
pub fn rk4_step_scalar<F>(t: f32, y: f32, dt: f32, mut f: F) -> Result<f32, Rk4Error>
where
    F: FnMut(f32, f32) -> f32,
{
    validate_step(t, dt)?;
    if !y.is_finite() {
        return Err(Rk4Error::NonFinite);
    }
    let half_dt = 0.5 * dt;
    let k1 = f(t, y);
    let k2 = f(t + half_dt, y + half_dt * k1);
    let k3 = f(t + half_dt, y + half_dt * k2);
    let k4 = f(t + dt, y + dt * k3);
    if !k1.is_finite() || !k2.is_finite() || !k3.is_finite() || !k4.is_finite() {
        return Err(Rk4Error::NonFinite);
    }
    let next = y + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    if !next.is_finite() {
        return Err(Rk4Error::NonFinite);
    }
    Ok(next)
}

#[inline]
pub(crate) fn accumulate_rk4<const N: usize>(
    y: &[f32; N],
    k1: &[f32; N],
    k2: &[f32; N],
    k3: &[f32; N],
    k4: &[f32; N],
    dt: f32,
) -> Result<[f32; N], Rk4Error> {
    let mut out = [0.0f32; N];
    for i in 0..N {
        let sum = k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i];
        if !sum.is_finite() {
            return Err(Rk4Error::NonFinite);
        }
        out[i] = y[i] + (dt / 6.0) * sum;
        if !out[i].is_finite() {
            return Err(Rk4Error::NonFinite);
        }
    }
    Ok(out)
}
