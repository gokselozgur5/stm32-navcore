use stm32_navcore::filters::alpha_beta::{AlphaBetaError, AlphaBetaFilter};

#[test]
fn parameter_validation_rejects_invalid_values() {
    assert_eq!(
        AlphaBetaFilter::new(0.5, 0.1, 0.0),
        Err(AlphaBetaError::InvalidSampleInterval)
    );
    assert_eq!(
        AlphaBetaFilter::new(0.5, 0.1, -0.01),
        Err(AlphaBetaError::InvalidSampleInterval)
    );
    assert_eq!(
        AlphaBetaFilter::new(0.0, 0.1, 0.01),
        Err(AlphaBetaError::InvalidAlpha)
    );
    assert_eq!(
        AlphaBetaFilter::new(1.5, 0.1, 0.01),
        Err(AlphaBetaError::InvalidAlpha)
    );
    assert_eq!(
        AlphaBetaFilter::new(0.5, -0.1, 0.01),
        Err(AlphaBetaError::InvalidBeta)
    );
}
