use stm32_navcore::gnss_dop::{compute_dop, DopError, SatelliteLos};

// Four unknowns (east, north, up, clock) need four independent rows.
// Three distinct directions give a rank 3 geometry matrix.
#[test]
fn duplicated_satellite_is_singular_geometry() {
    let a = SatelliteLos::from_azimuth_elevation(0.3, 0.4).unwrap();
    let b = SatelliteLos::from_azimuth_elevation(2.1, 0.9).unwrap();
    let c = SatelliteLos::from_azimuth_elevation(4.0, 1.2).unwrap();
    assert_eq!(compute_dop(&[a, b, c, c]), Err(DopError::SingularGeometry));
}
