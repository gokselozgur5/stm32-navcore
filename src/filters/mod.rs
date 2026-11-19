//! Discrete filters and signal conditioners.

pub mod alpha_beta;
pub mod biquad;
pub mod complementary;
pub mod lowpass;
pub mod median;
pub mod notch;
pub mod slew;

pub use alpha_beta::{AlphaBetaError, AlphaBetaFilter};
pub use biquad::{BiquadCoefficients, BiquadError, BiquadFilter};
pub use complementary::{ComplementaryError, ComplementaryFilter};
pub use lowpass::{LowPassError, LowPassFilter};
pub use median::MedianFilter;
pub use notch::NotchFilter;
pub use slew::{SlewError, SlewRateLimiter, SymmetricDeadband};
