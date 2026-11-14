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
        Ok(Quaternion::new(
            q0.w * f32::cos(theta) + q1.w * f32::sin(theta),
            q0.x * f32::cos(theta) - q1.x * f32::sin(theta),
            q0.y * f32::cos(theta) - q1.y * f32::sin(theta),
            q0.z * f32::cos(theta) + q1.z * f32::sin(theta),
        ))
    } else {
        // Collinear inputs fallback to nlerp
        Ok(nlerp(q0, q1, t))
    }
}

/// Normalized linear interpolation (nlerp) for attitude orientation quaternions.
///
/// # Examples
///
/// ```
/// use stm32_navcore::attitude::{nlerp, AttitudeError};
///
/// let q0 = Quaternion::new(1.0, 0.0, 0.0, 0.0);
/// let q1 = Quaternion::new(0.0, 1.0, 0.0, 0.0);
/// let t = 0.5;
///
/// match nlerp(q0, q1, t) {
///     Ok(quat) => println!("Nlerped quaternion: {:?}", quat),
///     Err(e) => println!("Error: {:?}", e),
/// }
/// ```
pub fn nlerp(q0: Quaternion, q1: Quaternion, t: f32) -> Result<Quaternion, AttitudeError> {
    let dot = q0.dot(&q1);
    if dot < 0.0 {
        // Shortest path across the quaternion double cover
        let theta = -f32::acos(dot);
        let sin_theta = f32::sin(theta);
        Ok(Quaternion::new(
            q0.w * f32::cos(theta) + q1.w * f32::sin(theta),
            q0.x * f32::cos(theta) - q1.x * f32::sin(theta),
            q0.y * f32::cos(theta) - q1.y * f32::sin(theta),
            q0.z * f32::cos(theta) + q1.z * f32::sin(theta),
        ))
    } else {
        // Collinear inputs fallback to nlerp
        Ok(Quaternion::new(
            q0.w * (1.0 - t) + q1.w * t,
            q0.x * (1.0 - t) + q1.x * t,
            q0.y * (1.0 - t) + q1.y * t,
            q0.z * (1.0 - t) + q1.z * t,
        ))
