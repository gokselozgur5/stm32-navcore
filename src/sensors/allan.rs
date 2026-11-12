//! Streaming overlapping Allan variance estimator for IMU noise.
//!
//! Rate samples are integrated into a phase record kept in a fixed ring
//! buffer. Averaging factors are octave spaced (m = 1, 2, 4, ...).

/// Overlapping Allan variance over `L` octave levels, with a phase ring of `N`.
pub struct AllanEstimator<const N: usize, const L: usize> {
    tau0: f64,
    phase: [f64; N],
    acc: f64,
    samples: u64,
    sum: [f64; L],
    count: [u64; L],
}
