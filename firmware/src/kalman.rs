use embassy_time::Timer;
use micromath::F32Ext;
use nalgebra::{Matrix3, Quaternion, UnitQuaternion, Vector3};

pub struct KalmanFilterQuat {
    pub q: UnitQuaternion<f32>,
    pub p: Matrix3<f32>,
    pub q_noise: f32,
    pub r_measure: f32,
}

impl KalmanFilterQuat {
    pub fn new() -> Self {
        Self {
            q: UnitQuaternion::identity(),
            p: Matrix3::identity() * 0.01,
            q_noise: 0.0100,
            r_measure: 0.0015,
        }
    }

    pub fn update(&mut self, gyro: Vector3<f32>, accel: Vector3<f32>, dt: f32) {
        let omega = Quaternion::new(0.0, gyro.x, gyro.y, gyro.z);
        let dq = 0.5 * self.q.as_ref() * omega;
        let pred_q = Quaternion::new(
            self.q.w + dq.w * dt,
            self.q.i + dq.i * dt,
            self.q.j + dq.j * dt,
            self.q.k + dq.k * dt,
        )
        .normalize();
        self.q = UnitQuaternion::from_quaternion(pred_q);

        let accel_norm = accel.normalize();
        let roll = accel_norm.y.atan2(accel_norm.z);
        let pitch = (-accel_norm.x).atan2((accel_norm.y.powi(2) + accel_norm.z.powi(2)).sqrt());

        let yaw = self.q.euler_angles().2; // Preserve yaw from gyro integration
        let accel_q = UnitQuaternion::from_euler_angles(roll, pitch, yaw);

        self.q = self.q.slerp(&accel_q, self.r_measure);
    }
}
