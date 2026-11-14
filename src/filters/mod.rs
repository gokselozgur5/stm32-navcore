//! Discrete filters and signal conditioners.

pub mod median;

pub use median::MedianFilter;
pub mod lowpass;

pub use lowpass::{LowPassError, LowPassFilter};
