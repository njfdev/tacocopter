use nalgebra::ComplexField;

use crate::tc_println;

// in degrees celsius per meter
pub const NOMINAL_LAPSE_RATE: f32 = -0.0065;

pub struct BaroErrorState {
    bias: f64,
    bias_var: f64,
    scale: f64,
    scale_var: f64,
}

impl BaroErrorState {
    pub fn new() -> Self {
        Self {
            bias: 0.0,
            bias_var: 1.0,
            scale: 0.0,
            scale_var: 1.0,
        }
    }

    pub fn propagate_state(&mut self, q_bias: f64, q_scale: f64) {
        self.bias_var += q_bias;
        self.scale_var += q_scale;
    }

    pub fn temperature_update(
        &mut self,
        lambda: f64,
        pred_height: f64,
        temp: f64,
        lapse_rate: f64,
        t0: f64,
        h0: f64,
        r_lambda: f64,
    ) {
        let expected_lambda = (temp - (t0 + lapse_rate * (pred_height - h0))) / t0;
        let innovation = lambda - expected_lambda;
        let kalman_gain = self.scale_var / (self.scale_var + r_lambda);
        self.scale += kalman_gain * innovation;
        self.scale_var *= 1.0 - kalman_gain;
    }

    pub fn gps_update(&mut self, gps_height: f64, raw_baro: f64, r_gps: f64, r_baro: f64) {
        let corrected_baro = self.correct_height(raw_baro);
        let innovation = gps_height - corrected_baro;
        let total_var = r_gps + r_baro;

        // Update scale and bias jointly using simplified gains
        let gain_scale = self.scale_var / (self.scale_var + total_var);
        let gain_bias = self.bias_var / (self.bias_var + total_var);

        // These update rules ensure correct direction of correction
        let denom = (1.0 + self.scale).powi(2);
        self.scale += gain_scale * innovation * raw_baro / denom;
        self.scale_var *= 1.0 - gain_scale;

        self.bias += gain_bias * innovation / (1.0 + self.scale);
        self.bias_var *= 1.0 - gain_bias;

        // tc_println!("gps_height: {}", gps_height);
        // tc_println!("raw_baro: {}", raw_baro);
        // tc_println!("corrected_baro: {}", corrected_baro);
        // tc_println!("innovation: {}", innovation);
        // tc_println!("gain_scale: {}", gain_scale);
        // tc_println!("gain_bias: {}", gain_bias);
        // tc_println!("scale_var: {}", (self.scale_var));
        // tc_println!("bias_var: {}", (self.bias_var));
        // tc_println!("bias: {}, scale: {}", (self.bias), (self.scale));
    }

    pub fn correct_height(&self, raw_baro: f64) -> f64 {
        (raw_baro - self.bias) / (1.0 + self.scale)
    }
}
