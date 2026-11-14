use stm32_navcore::filters::ComplementaryFilter;

#[test]
fn test_complementary_filter_construction_and_step() {
    let filter = ComplementaryFilter::new(0.98, 0.01).unwrap();
    assert_eq!(filter.roll(), 0.0);
    assert_eq!(filter.pitch(), 0.0);
}
