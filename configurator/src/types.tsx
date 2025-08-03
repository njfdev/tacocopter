export type Message =
  | { State: State }
  | { ImuSensor: ImuSensor }
  | { Sensor: Sensor }
  | { SensorCalibration: SensorCalibration | string }
  | { ElrsChannels: ElrsChannels }
  | { Log: LogLine[] }
  | { PIDSettings: PIDSettings }
  | { Blackbox: boolean };

export interface State {
  target_update_rate: number;
  imu_process_rate: number;
  control_loop_update_rate: number;
  position_hold_loop_update_rate: number;
  uptime: number;
}

export interface PIDSettings {
  pitch: [number, number, number];
  roll: [number, number, number];
  yaw: [number, number, number];
}

export interface ImuSensor {
  gyroscope: [number, number, number];
  accelerometer: [number, number, number];
  // gyro_orientation: [number, number, number, number];
  // accel_orientation: [number, number, number, number];
  // orientation: [number, number, number, number];
}

export interface Sensor {
  estimated_altitude: number;
  ultrasonic_dist: number;
}

export interface SensorCalibration {
  Data: {
    gyro_calibration: [number, number, number];
    accel_calibration: [number, number, number];
  };
  GyroProgress: {
    samples: number;
    seconds_remaining: number;
  };
}

export type ElrsChannels = [
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number,
  number
];

export interface LogLine {
  id: number;
  level: string;
  text: string;
}

export interface TCData {
  state: State;
  imuSensors: ImuSensor;
  sensors: Sensor;
  sensorCalibration: SensorCalibration;
  channels: ElrsChannels;
  logs: LogLine[];
  pid: PIDSettings;
  blackbox_enabled: boolean;
}
