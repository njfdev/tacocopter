pub struct BarometerAltitudePacket {
    // in meters
    pub altitude: f32,
    // in m/s
    pub vertical_speed: f32,
}

pub struct BatteryStatePacket {
    pub voltage: f32,
    pub current: f32,
    pub capacity: u32,
    pub battery_percentage: f32,
}

pub struct GPSPacket {
    pub latitude: f64,
    pub longitude: f64,
    pub ground_speed: f32,
    pub gps_heading: f32,
    pub gps_altitude: f32,
    pub num_sats: u8,
}
