use stm32_navcore::ubx::{CarrierSolution, UbxNavPvt, UbxNavRelPosNed};

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

fn rel_with_flags(flags: u32) -> UbxNavRelPosNed {
    let mut p = [0u8; 64];
    p[60..64].copy_from_slice(&flags.to_le_bytes());
    UbxNavRelPosNed::decode(&p).expect("valid RELPOSNED")
}

#[test]
fn relposned_carrier_solution_and_validity() {
    assert_eq!(rel_with_flags(0).carrier_solution(), CarrierSolution::None);
    assert_eq!(
        rel_with_flags(1 << 3).carrier_solution(),
        CarrierSolution::Float
    );
    assert_eq!(
        rel_with_flags(2 << 3).carrier_solution(),
        CarrierSolution::Fixed
    );
    assert_eq!(
        rel_with_flags(3 << 3).carrier_solution(),
        CarrierSolution::None
    );
    let fixed = rel_with_flags((2 << 3) | (1 << 2) | (1 << 8));
    assert!(fixed.rel_pos_valid() && fixed.heading_valid());
    assert!(!rel_with_flags(!((1 << 2) | (1 << 8))).rel_pos_valid());
    assert!(!rel_with_flags(!(1 << 8)).heading_valid());
}
