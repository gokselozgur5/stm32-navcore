use stm32_navcore::linalg::{Ud, UdReconstruct};

#[test]
fn test_ud_reconstruction() {
    let cov = [
        [1.0, 0.5, 0.3, 0.2],
        [0.5, 1.0, 0.4, 0.3],
        [0.3, 0.4, 1.0, 0.2],
        [0.2, 0.3, 0.2, 1.0],
    ];

    let ud = Ud::new(&cov);
    let reconstructed_cov = ud.reconstruct();

    assert_eq!(reconstructed_cov, cov);
}

#[test]
fn test_ud_add_scalar() {
    let mut ud = Ud::new(&[[1.0, 0.5], [0.5, 1.0]]);
    ud.add_scalar(2.0);

    assert_eq!(ud.u[0], 3.0);
    assert_eq!(ud.d[0], 3.0);
}

#[test]
fn test_ud_div_scalar() {
    let mut ud = Ud::new(&[[1.0, 0.5], [0.5, 1.0]]);
    ud.div_scalar(2
