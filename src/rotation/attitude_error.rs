use std::f32;

/// Calculates the attitude error angle between two quaternions.
///
/// The attitude error angle is defined as the difference between the applied test rotation and the quaternion
/// representing the current attitude. The result is invariant to quaternion sign.
///
/// # Arguments
///
/// * `q1` - The first quaternion representing the current attitude.
/// * `q2` - The second quaternion representing the applied test rotation.
///
/// # Returns
///
/// * The attitude error angle in radians.
pub fn attitude_error_angle(q1: &quaternion::Quaternion<f32>, q2: &quaternion::Quaternion<f32>) -> f32 {
    let dot_product = q1.dot(q2);
    let cos_theta = dot_product / (q1.norm() * q2.norm());
    let theta = f32::acos(cos_theta);

    // Calculate the axis of rotation
    let mut axis = quaternion::Vector3<f32>::new(0.0, 0.0, 0.0);
    if dot_product != 1.0 && dot_product != -1.0 {
        let cross_product = q1.cross(q2);
