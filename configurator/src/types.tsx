export type Message =
  | { State: State }
  | { ImuSensor: ImuSensor }
  | { Sensor: Sensor }
  | { SensorCalibration: SensorCalibration }
  | { ElrsChannels: ElrsChannels }
  | { Log: LogData };

export interface State {
  sensor_update_rate: number;
}

export interface ImuSensor {
  // gyroscope: [number, number, number];
  // accelerometer: [number, number, number];
  gyro_orientation: [number, number, number, number];
  accel_orientation: [number, number, number, number];
  orientation: [number, number, number, number];
}

export interface Sensor {
  estimated_altitude: number;
  ultrasonic_dist: number;
}

export interface SensorCalibration {
  gyro_calibration: [number, number, number];
  accel_calibration: [number, number, number];
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

export interface LogData {
  id: number;
  text: string;
}

export interface TCData {
  state: State;
  imuSensors: ImuSensor;
  sensors: Sensor;
  sensorCalibration: SensorCalibration;
  channels: ElrsChannels;
  log: LogData;
}
