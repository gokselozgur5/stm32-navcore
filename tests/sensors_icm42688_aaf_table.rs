use stm32_navcore::sensors::icm42688::AafConfig;

#[test]
fn aaf_536hz_matches_datasheet_row() {
    let c = AafConfig::cutoff_536hz();
    assert_eq!((c.delt, c.deltsqr, c.bitshift), (12, 144, 8));
}
