use stm32_navcore::filters::MedianFilter;

#[test]
fn test_median_filter_odd_window() {
    let mut f = MedianFilter::<5>::new();
    assert!(!f.is_full());
    assert_eq!(f.update(10.0), 10.0);
    assert_eq!(f.update(2.0), 10.0);
    assert_eq!(f.update(5.0), 5.0);
    assert_eq!(f.update(100.0), 10.0);
    assert_eq!(f.update(1.0), 5.0);
    assert!(f.is_full());
    assert_eq!(f.update(6.0), 5.0);
}

#[test]
fn test_median_filter_reset() {
    let mut f = MedianFilter::<3>::new();
    f.update(1.0);
    f.update(2.0);
    f.reset();
    assert!(!f.is_full());
    assert_eq!(f.update(42.0), 42.0);
}

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

#[test]
fn test_median_filter_copy_and_equality() {
    let mut f1 = MedianFilter::<3>::new();
    f1.init_with(5.0);
    let f2 = f1;
    assert_eq!(f1, f2);
}
