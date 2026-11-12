use stm32_navcore::sensors::allan::AllanEstimator;

pub struct Rng(u64);

impl Rng {
    pub fn next_u(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    pub fn gauss(&mut self) -> f64 {
        let (a, b) = (self.next_u(), self.next_u());
        libm::sqrt(-2.0 * libm::log(a)) * libm::cos(2.0 * core::f64::consts::PI * b)
    }
}

#[test]
fn white_noise_matches_theory() {
    let (tau0, sigma) = (0.01, 0.5);
    let mut est = AllanEstimator::<129, 7>::new(tau0);
    let mut rng = Rng(0x9E37_79B9_7F4A_7C15);
    for _ in 0..200_000 {
        est.push(sigma * rng.gauss());
    }
    for level in 0..7 {
        let theory = sigma * libm::sqrt(tau0 / est.tau(level));
        let got = est.adev(level).unwrap();
        assert!(
            (got / theory - 1.0).abs() < 0.08,
            "level {level}: {got} vs {theory}"
        );
    }
}

#[test]
fn constant_rate_has_zero_variance() {
    let mut est = AllanEstimator::<17, 3>::new(0.1);
    for _ in 0..100 {
        est.push(2.5);
    }
    for level in 0..3 {
        assert!(est.avar(level).unwrap() < 1e-18);
    }
}

#[test]
fn counts_follow_tau_schedule() {
    let mut est = AllanEstimator::<17, 3>::new(0.5);
    assert!(est.adev(0).is_none());
    for _ in 0..20 {
        est.push(1.0);
    }
    assert_eq!(est.tau(2), 2.0);
    assert_eq!((est.count(0), est.count(1), est.count(2)), (19, 17, 13));
}

#[test]
#[should_panic]
fn undersized_ring_panics() {
    let _ = AllanEstimator::<8, 3>::new(1.0);
}
