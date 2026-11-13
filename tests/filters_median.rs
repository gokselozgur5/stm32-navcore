use stm32_navcore::filters::MedianFilter;

#[test]
fn single_sample_spikes_are_removed() {
    let mut med = MedianFilter::<5>::new();
    med.init_with(10.0);
    assert_eq!(med.update(100.0), 10.0);
    assert_eq!(med.update(10.0), 10.0);
    assert_eq!(med.update(-50.0), 10.0);
    assert_eq!(med.update(10.0), 10.0);
}

#[test]
fn step_edge_is_delayed_by_exactly_half_window_samples() {
    let mut med5 = MedianFilter::<5>::new();
    med5.init_with(0.0);
    assert_eq!(med5.update(1.0), 0.0);
    assert_eq!(med5.update(1.0), 0.0);
    assert_eq!(med5.update(1.0), 1.0);

    let mut med7 = MedianFilter::<7>::new();
    med7.init_with(0.0);
    assert_eq!(med7.update(1.0), 0.0);
    assert_eq!(med7.update(1.0), 0.0);
    assert_eq!(med7.update(1.0), 0.0);
    assert_eq!(med7.update(1.0), 1.0);
}
