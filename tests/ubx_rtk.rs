use stm32_navcore::ubx::{CarrierSolution, UbxNavPvt};

fn pvt_with_flags(flags: u8) -> UbxNavPvt {
    let mut p = [0u8; 92];
    p[21] = flags;
    UbxNavPvt::decode(&p).expect("valid PVT")
}

#[test]
fn pvt_carrier_solution_covers_all_values() {
    let cases = [
        (0x00, CarrierSolution::None),
        (0x40, CarrierSolution::Float),
        (0x80, CarrierSolution::Fixed),
        (0xC0, CarrierSolution::None),
    ];
    for (flags, expected) in cases {
        assert_eq!(pvt_with_flags(flags).carrier_solution(), expected);
    }
}

#[test]
fn pvt_gnss_fix_ok_reads_bit_zero() {
    assert!(pvt_with_flags(0x01).gnss_fix_ok());
    assert!(!pvt_with_flags(0xFE).gnss_fix_ok());
}
