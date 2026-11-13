//! Second-order notch filter for motor harmonic rejection.

use crate::filters::Biquad;

/// Second-order notch filter built upon a DF2T biquad section.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NotchFilter {
    biquad: Biquad,
}
