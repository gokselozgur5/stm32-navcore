use stm32_navcore::butterworth::ButterworthFilter;

#[test]
fn low_cutoff_ratio_is_rejected_or_stays_bounded() {
    if let Ok(mut filter) = ButterworthFilter::try_new(0.05, 0.001) {
        let mut peak = 0.0f32;
        for _ in 0..40_000 {
            let y = filter.update(1.0).abs();
            if y > peak {
                peak = y;
            }
        }
        assert!(peak <= 1.1, "step response peak = {peak}");
    }
}
