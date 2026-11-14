use stm32_navcore::filters::MedianFilter;

#[test]
fn test_median_filter_odd_window() {
    let mut f = MedianFilter::<5>::new();
    assert!(!f.is_full());
    assert_eq!(f.update(10.0), 10.0);
    assert_eq!(f.update(2.0), 10.0); // sorted [2, 10] -> index 1 is 10
    assert_eq!(f.update(5.0), 5.0); // sorted [2, 5, 10] -> index 1 is 5
    assert_eq!(f.update(100.0), 10.0); // sorted [2, 5, 10, 100] -> index 2 is 10
    assert_eq!(f.update(1.0), 5.0); // sorted [1, 2, 5, 10, 100] -> index 2 is 5
    assert!(f.is_full());

    // Next sample evicts oldest (10.0), window has [2, 5, 100, 1, 6] -> sorted [1, 2, 5, 6, 100] -> 5
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
