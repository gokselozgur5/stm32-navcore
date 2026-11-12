use stm32_navcore::sensors::mag_cal::{MagCal, MagMinMax};

fn fill() -> MagMinMax {
    let mut mm = MagMinMax::new();
    for s in [
        [30.0, 5.0, -20.0],
        [-10.0, 5.0, -20.0],
        [10.0, 45.0, -20.0],
        [10.0, -35.0, -20.0],
        [10.0, 5.0, 20.0],
        [10.0, 5.0, -60.0],
    ] {
        mm.update(s);
    }
    mm
}

#[test]
fn offset_is_midpoint() {
    assert_eq!(fill().offset(), [10.0, 5.0, -20.0]);
    assert_eq!(MagMinMax::new().offset(), [0.0; 3]);
}

