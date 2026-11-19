use stm32_navcore::linalg::lstsq::lstsq;
use stm32_navcore::linalg::LinalgError;
use stm32_navcore::math::Mat;

#[test]
fn rank_deficient_system_with_large_entries_is_rejected() {
    let a = Mat([[1000.0f32, 2000.0], [3000.0, 6000.0], [5000.0, 10000.0]]);
    assert_eq!(lstsq(&a, &[1.0, 2.0, 3.0]), Err(LinalgError::ZeroPivot));
}

#[test]
fn full_rank_system_with_small_entries_is_solved() {
    let a = Mat([[1.0e-7f32, 0.0], [0.0, 1.0e-7], [0.0, 0.0]]);
    let x = lstsq(&a, &[1.0e-7, 2.0e-7, 0.0]).expect("full column rank");
    assert!((x[0] - 1.0).abs() < 1.0e-4 && (x[1] - 2.0).abs() < 1.0e-4);
}
