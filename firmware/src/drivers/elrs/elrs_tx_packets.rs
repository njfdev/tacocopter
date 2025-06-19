pub struct BarometerAltitudePacket {
    // in meters
    baro_altitude: f32,
}

pub struct BatteryStatePacket {
    voltage: f32,
    current: f32,
    battery_percentage: f32,
}

pub struct GPSPacket {
    latitude: f64,
    longitude: f64,
    ground_speed: f32,
    gps_heading: f32,
    gps_altitude: f32,
    num_sats: u8,
}
