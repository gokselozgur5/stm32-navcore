//! Statistical consistency checks for the navigation EKF.

use stm32_navcore::ekf::NavEkf;

/// Deterministic LCG with a Box-Muller Gaussian transform.
struct Rng(u64);

impl Rng {
    fn next_u32(&mut self) -> u32 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 33) as u32
    }

    fn uniform(&mut self) -> f32 {
        (self.next_u32() as f32 + 1.0) / 2147483649.0
    }

    fn gauss(&mut self) -> f32 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        (-2.0 * u1.ln()).sqrt() * (core::f32::consts::TAU * u2).cos()
    }
}
