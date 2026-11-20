use stm32_navcore::protocol::rc_esc::CrsfFrame;
use stm32_navcore::wind_estimator::{wind_speed_and_direction, wrap_2pi};

#[test]
fn wind_from_direction_stays_below_360_degrees() {
    assert!(wrap_2pi(-1e-7) < 2.0 * core::f32::consts::PI);
    let info = wind_speed_and_direction([-10.0, 1e-6]).unwrap();
    assert!(
        info.direction_from_deg < 360.0,
        "{}",
        info.direction_from_deg
    );
}

#[test]
fn normalized_channel_stays_in_unit_range() {
    let low = CrsfFrame {
        raw_channels: [0; 16],
    }
    .normalized_channel(0);
    let high = CrsfFrame {
        raw_channels: [2047; 16],
    }
    .normalized_channel(0);
    assert!(
        (-1.0..=1.0).contains(&low) && (-1.0..=1.0).contains(&high),
        "{low} {high}"
    );
}
