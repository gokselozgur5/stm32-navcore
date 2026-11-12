use stm32_navcore::sensors::baro::{compensated_altitude_m, BaroMeasurement, ISA_SEA_LEVEL_HPA};

#[test]
fn isa_conditions_leave_altitude_unchanged() {
    for p in [898.7_f32, 701.1, 540.5] {
        let isa = BaroMeasurement::from_pressure(p, ISA_SEA_LEVEL_HPA, 1.0)
            .unwrap()
            .altitude_m;
        let temp_c = 15.0 - 0.0065 * isa;
        let alt = compensated_altitude_m(p, ISA_SEA_LEVEL_HPA, temp_c).unwrap();
        assert!((alt - isa).abs() < 0.5, "isa {isa} compensated {alt}");
    }
}
