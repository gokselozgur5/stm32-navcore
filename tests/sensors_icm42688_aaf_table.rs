use stm32_navcore::sensors::icm42688::AafConfig;

// ICM-42688-P datasheet DS-000347, section 5.3 anti-alias filter table:
// 536 Hz -> AAF_DELT 12, AAF_DELTSQR 144, AAF_BITSHIFT 8.
#[test]
fn aaf_536hz_matches_datasheet_row() {
    let c = AafConfig::cutoff_536hz();
    assert_eq!((c.delt, c.deltsqr, c.bitshift), (12, 144, 8));
}

#[test]
fn aaf_258hz_matches_datasheet_row() {
    let c = AafConfig::cutoff_258hz();
    assert_eq!((c.delt, c.deltsqr, c.bitshift), (6, 36, 10));
}
