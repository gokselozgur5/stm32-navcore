//! Mathematical primitives: matrix algebra, quaternions, kinematics, RK4, rotation, and linear algebra.

pub mod body_rot;
pub mod core;
pub mod kinematics;
pub mod linalg;
pub mod rk4;
pub mod rotation;

pub use body_rot::{BodyRot, BodyRotError, BODY_ROT_TOLERANCE};
pub use core::{
    calibrate_magnetometer, pressure_hpa_to_altitude_m, tilt_compensated_yaw, Mat, MathError,
    Quaternion, Vec3,
};
pub use kinematics::{
    attitude_transition_matrix, body_rates_to_euler_rates, body_to_euler_rates_matrix,
    euler_rates_to_body_matrix, euler_rates_to_body_rates, euler_rates_to_quat_rates, omega_matrix,
    propagate_attitude_quaternion, propagate_euler_angles, quat_derivative,
    quat_derivative_to_body_rates, quat_rates_to_euler_rates, xi_matrix, EulerAngles,
    KinematicsError, GIMBAL_LOCK_COS_THRESHOLD,
};
pub use rk4::{rk4_integrate_scalar, rk4_step, rk4_step_scalar, Rk4Error};
