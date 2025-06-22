// TODO: have a way to calibrate this, and then store it in something like Flash memory
// update based on calibration data
pub const ACCEL_BIASES: [f32; 3] = [0.044174805, -0.063529054, 0.07425296];
pub const GYRO_BIASES: [f32; 3] = [-0.0356924, -0.0230041, -0.03341522];
pub const GRAVITY: f32 = 9.80665;

pub const UPDATE_LOOP_FREQUENCY: f64 = 200.0;
pub const USB_LOGGER_RATE: f32 = 30.0;

// in m
pub const ULTRASONIC_HEIGHT_ABOVE_BOTTOM: f32 = 0.1515;
// the offset of the ultrasonic sensor from the center that is corrected by pitch
pub const ULTRASONIC_DISTANCE_TO_CENTER_PITCH: f32 = 0.093;
