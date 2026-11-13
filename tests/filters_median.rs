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
