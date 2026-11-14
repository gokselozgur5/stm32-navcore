//! Second-order notch filter for motor harmonic rejection (NC-019).

use crate::filters::{BiquadCoefficients, BiquadFilter};

/// Second-order notch filter built upon a DF2T biquad section.
#[derive(Clone, Copy, Debug)]
pub struct NotchFilter {
    biquad: BiquadFilter,
}
