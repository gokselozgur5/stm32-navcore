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
