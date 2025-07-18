use embassy_time::Instant;

use crate::{
    consts::{GRAVITY, UPDATE_LOOP_FREQUENCY},
    tools::moving_average::F32MovingAverage,
};
const AVERAGE_WINDOW_SIZE: usize = UPDATE_LOOP_FREQUENCY as usize / 20;

// This value is determined after accel calibration, based on the quadcopter sitting flat on its landing gear
// TODO: should be a part of accel calibration process
pub const ACCEL_VERTICAL_BIAS: f32 = 0.003;

pub struct AltitudeEstimator {
    // estimated values
    altitude_msl: f32,
    vertical_vel: F32MovingAverage<AVERAGE_WINDOW_SIZE>,

    accel_vertical_vel: f32,
    last_altitude_msl: f32,

    // state values
    baro_offset: f32, // constantly adjusted by GPS (if GPS is lost, it will stay the same but the barometer will drift)
    last_baro_height: f32, // used to estimate barometer vertical speed
    ground_offset: f32, // based on ultrasonic sensor and used to calculate AGL even when ultrasonic is out of range

    last_accel_update: Instant,
    last_baro_update: Instant,
    last_altitude_update: Instant,
}

impl AltitudeEstimator {
    pub fn new() -> Self {
        Self {
            altitude_msl: 0.0,
            vertical_vel: F32MovingAverage::new(),

            accel_vertical_vel: 0.0,
            last_altitude_msl: 0.0,

            baro_offset: 0.0,
            last_baro_height: 0.0,
            ground_offset: 0.0,

            last_accel_update: Instant::now(),
            last_baro_update: Instant::now(),
            last_altitude_update: Instant::now(),
        }
    }

    // Only call when arming and the drone is not moving and there is an accurate satellite fix (e.g., sats > 10 and vertical accuracy <1m), and the ultrasonic sensor is 0m
    pub fn reset(&mut self, barometer_altitude: f32, gps_altitude: f32) {
        self.altitude_msl = gps_altitude;
        self.vertical_vel.reset();

        self.accel_vertical_vel = 0.0;
        self.last_altitude_msl = gps_altitude;

        self.baro_offset = gps_altitude - barometer_altitude;
        self.last_baro_height = barometer_altitude + self.baro_offset;
        self.ground_offset = gps_altitude;

        self.last_accel_update = Instant::now();
        self.last_baro_update = Instant::now();
        self.last_altitude_update = Instant::now();
    }

    fn altitude_update(&mut self) {
        let elapsed = self.last_altitude_update.elapsed().as_micros() as f32;
        if elapsed < (1_000_000.0 / (UPDATE_LOOP_FREQUENCY / 2.0) as f32) {
            return;
        }

        let dt = elapsed / 1_000_000.0;
        self.last_altitude_update = Instant::now();
        self.vertical_vel
            .sample((self.altitude_msl - self.last_altitude_msl) / dt);
        self.last_altitude_msl = self.altitude_msl;
    }

    // expects an acceleration in Gs with the gravity vector removed and is compensation for orientation
    pub fn update_accel(&mut self, corrected_accel: f32) {
        let dt = (self.last_accel_update.elapsed().as_micros() as f32) / 1_000_000.0;
        self.last_accel_update = Instant::now();
        self.accel_vertical_vel += (corrected_accel - ACCEL_VERTICAL_BIAS) * dt * GRAVITY;
        self.altitude_msl += self.accel_vertical_vel * dt;
        self.altitude_update();
    }

    pub fn update_ultrasonic(&mut self, ultrasonic_dist: f32) {
        self.ground_offset = self.altitude_msl - ultrasonic_dist;
    }

    pub fn update_barometer(&mut self, barometer_altitude: f32) {
        // get deltaTime
        let dt = (self.last_baro_update.elapsed().as_micros() as f32) / 1_000_000.0;
        self.last_baro_update = Instant::now();

        // get corrected barometer altitude
        let corrected = self.baro_offset + barometer_altitude;

        // calc altitude difference, and move altitude to be more accurate
        self.altitude_msl += (corrected - self.altitude_msl) * 0.05;

        // calc vertical speed, vertical speed difference, and move vertical speed to be more accurate
        let baro_vs = (corrected - self.last_baro_height) / dt;
        self.accel_vertical_vel += (baro_vs - self.accel_vertical_vel) * 0.05;
        self.last_baro_height = corrected;
        self.altitude_update();
    }

    // updates absolute altitude, vertical speed, and makes corrects to barometer offset
    pub fn update_gps(&mut self, gps_altitude: f32, gps_vertical_speed: f32) {
        // calc altitude difference, and move altitude to be more accurate
        self.altitude_msl += (gps_altitude - self.altitude_msl) * 0.0004;

        // calc vertical speed difference, and move vertical speed to be more accurate
        self.accel_vertical_vel += (gps_vertical_speed - self.accel_vertical_vel) * 0.0004;

        // calc altitude difference to barometer, and move barometer offset to be more accurate
        self.baro_offset += (gps_altitude - self.last_baro_height) * 0.0004;
        self.altitude_update();
    }

    pub fn get_altitude_msl(&self) -> f32 {
        self.altitude_msl
    }

    pub fn get_altitude_agl(&self) -> f32 {
        self.altitude_msl - self.ground_offset
    }

    pub fn get_vertical_velocity(&self) -> f32 {
        self.vertical_vel.get_average()
    }
}
