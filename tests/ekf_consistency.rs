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

const GRAVITY_UP: [f32; 3] = [0.0, 0.0, -9.80665];

/// Stationary run at the origin; returns every scalar NIS after warm-up.
fn stationary_nis(seed: u64, steps: usize, warmup: usize) -> Vec<f32> {
    let mut rng = Rng(seed);
    let mut ekf = NavEkf::new();
    let mut out = Vec::new();
    for k in 0..steps {
        ekf.predict(GRAVITY_UP, [0.0; 3], 0.1).unwrap();
        let z = [rng.gauss(), rng.gauss(), rng.gauss()];
        let nis = ekf.update_gnss_pos(z, 1.0, 100.0).unwrap();
        if k >= warmup {
            out.extend_from_slice(&nis);
        }
    }
    out
}

#[test]
fn rng_is_deterministic_and_unit_variance() {
    let mut a = Rng(7);
    let mut b = Rng(7);
    let n = 20000;
    let mut sum_sq = 0.0f64;
    for _ in 0..n {
        let x = a.gauss();
        assert_eq!(x, b.gauss());
        sum_sq += (x * x) as f64;
    }
    let var = sum_sq / n as f64;
    assert!((0.95..1.05).contains(&var), "variance {var}");
}

#[test]
fn mean_nis_is_chi_square_consistent() {
    let nis = stationary_nis(42, 2000, 200);
    let mean = nis.iter().map(|&v| v as f64).sum::<f64>() / nis.len() as f64;
    assert!((0.8..1.2).contains(&mean), "mean NIS {mean}");
}

#[test]
fn nis_exceedance_rate_matches_95_percent_gate() {
    let nis = stationary_nis(1234, 2000, 200);
    let over = nis.iter().filter(|&&v| v > 3.841).count();
    let rate = over as f64 / nis.len() as f64;
    assert!(rate < 0.07, "exceedance rate {rate}");
}

/// Stationary run; returns (x innovations, summed squared position error).
fn stationary_run(seed: u64, steps: usize) -> (Vec<f32>, Vec<f32>) {
    let mut rng = Rng(seed);
    let mut ekf = NavEkf::new();
    let (mut innov, mut err_sq) = (Vec::new(), Vec::new());
    for _ in 0..steps {
        ekf.predict([0.0, 0.0, -9.80665], [0.0; 3], 0.1).unwrap();
        let z = [rng.gauss(), rng.gauss(), rng.gauss()];
        innov.push(z[0] - ekf.p_ned_m[0]);
        ekf.update_gnss_pos(z, 1.0, 100.0).unwrap();
        let p = ekf.p_ned_m;
        err_sq.push(p[0] * p[0] + p[1] * p[1] + p[2] * p[2]);
    }
    (innov, err_sq)
}
