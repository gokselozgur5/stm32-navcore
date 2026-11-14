use core::f32;

/// Spherical linear interpolation (slerp) for attitude orientation quaternions.
///
/// # Examples
///
/// ```
/// use stm32_navcore::attitude::{slerp, AttitudeError};
///
/// let q0 = Quaternion::new(1.0, 0.0, 0.0, 0.0);
/// let q1 = Quaternion::new(0.0, 1.0, 0.0, 0.0);
/// let t = 0.5;
///
/// match slerp(q0, q1, t) {
///     Ok(quat) => println!("Slerped quaternion: {:?}", quat),
///     Err(e) => println!("Error: {:?}", e),
/// }
/// ```
pub fn slerp(q0: Quaternion, q1: Quaternion, t: f32) -> Result<Quaternion, AttitudeError> {
    let dot = q0.dot(&q1);
    if dot < 0.0 {
        // Shortest path across the quaternion double cover
        let theta = -f32::acos(dot);
        let sin_theta = f32::sin(theta);
