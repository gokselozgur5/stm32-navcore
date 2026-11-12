use stm32_navcore::sensors::allan::AllanEstimator;

struct Rng(u64);

impl Rng {
    fn next_u(&mut self) -> f64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        ((self.0 >> 11) as f64 + 0.5) / (1u64 << 53) as f64
    }

    fn gauss(&mut self) -> f64 {
        let (a, b) = (self.next_u(), self.next_u());
        libm::sqrt(-2.0 * libm::log(a)) * libm::cos(2.0 * core::f64::consts::PI * b)
    }
}

type Est = AllanEstimator<129, 7>;
const TAU0: f64 = 0.01;

#[test]
fn white_noise_coefficient_recovered() {
    let sigma = 0.5;
    let mut est = Est::new(TAU0);
    let mut rng = Rng(0x1234_5678_9ABC_DEF1);
    for _ in 0..200_000 {
        est.push(sigma * rng.gauss());
    }
    let n = est.white_noise_coeff(0, 5).unwrap();
    let theory = sigma * libm::sqrt(TAU0);
    assert!((n / theory - 1.0).abs() < 0.1, "{n} vs {theory}");
    let slope = est.loglog_slope(0, 5).unwrap();
    assert!((slope + 0.5).abs() < 0.05, "slope {slope}");
}

#[test]
fn random_walk_coefficient_recovered() {
    let k = 0.02;
    let mut est = Est::new(TAU0);
    let mut rng = Rng(0xDEAD_BEEF_CAFE_F00D);
    let mut rate = 0.0;
    for _ in 0..400_000 {
        rate += k * libm::sqrt(TAU0) * rng.gauss();
        est.push(rate);
    }
    let got = est.random_walk_coeff(2, 6).unwrap();
    assert!((got / k - 1.0).abs() < 0.1, "{got} vs {k}");
}

#[test]
fn invalid_ranges_return_none() {
    let mut est = Est::new(TAU0);
    assert!(est.white_noise_coeff(0, 3).is_none());
    est.push(1.0);
    est.push(2.0);
    assert!(est.white_noise_coeff(3, 1).is_none());
    assert!(est.random_walk_coeff(0, 7).is_none());
    assert!(est.loglog_slope(2, 2).is_none());
}
