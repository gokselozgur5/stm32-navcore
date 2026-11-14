use stm32_navcore::linalg::lstsq::lstsq;
use stm32_navcore::linalg::LinalgError;
use stm32_navcore::math::Mat;

#[test]
fn exact_square_system() {
    let a = Mat::<2, 2>([[2.0, 1.0], [1.0, 3.0]]);
    let x = lstsq(&a, &[5.0, 10.0]).unwrap();
    assert!((x[0] - 1.0).abs() < 1e-5 && (x[1] - 3.0).abs() < 1e-5);
}

#[test]
fn line_fit_overdetermined() {
    let a = Mat::<4, 2>([[1.0, 0.0], [1.0, 1.0], [1.0, 2.0], [1.0, 3.0]]);
    let x = lstsq(&a, &[1.0, 3.0, 5.0, 7.0]).unwrap();
    assert!((x[0] - 1.0).abs() < 1e-4 && (x[1] - 2.0).abs() < 1e-4);
}


