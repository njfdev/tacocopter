import { useEffect, useRef, useState } from "react";
import reactLogo from "./assets/react.svg";
import { invoke } from "@tauri-apps/api/core";
import "./App.css";
import { listen } from "@tauri-apps/api/event";
import { Canvas } from "@react-three/fiber";
import { OrbitControls, Text } from "@react-three/drei";

import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Title,
  Tooltip,
  Legend,
} from "chart.js";
import { Line } from "react-chartjs-2";
import { Euler, Quaternion } from "three";

type Message =
  | { State: State }
  | { ImuSensor: ImuSensor }
  | { Sensor: Sensor }
  | { SensorCalibration: SensorCalibration }
  | { ElrsChannels: ElrsChannels };

interface State {
  sensor_update_rate: number;
}

interface ImuSensor {
  // gyroscope: [number, number, number];
  // accelerometer: [number, number, number];
  gyro_orientation: [number, number, number, number];
  accel_orientation: [number, number, number, number];
  orientation: [number, number, number, number];
}

interface Sensor {
  estimated_altitude: number;
  ultrasonic_dist: number;
}

interface SensorCalibration {
  gyro_calibration: [number, number, number];
  accel_calibration: [number, number, number];
}

interface ElrsChannels {
  channels: [
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
}

function App() {
  // Register Chart.js components
  ChartJS.register(
    CategoryScale,
    LinearScale,
    PointElement,
    LineElement,
    Title,
    Tooltip,
    Legend
  );

  const [state, setState] = useState<State>({
    sensor_update_rate: 0,
  });
  const [imuSensors, setImuSensors] = useState<ImuSensor>({
    // gyroscope: [0, 0, 0],
    // accelerometer: [0, 0, 0],
    gyro_orientation: [0, 0, 0, 0],
    accel_orientation: [0, 0, 0, 0],
    orientation: [0, 0, 0, 0],
  });
  const [eulerImu, setEulerImu] = useState({
    gyro: [0.0, 0.0, 0.0],
    accel: [0.0, 0.0, 0.0],
    filtered: [0.0, 0.0, 0.0],
  });
  const [sensors, setSensors] = useState<Sensor>({
    estimated_altitude: 0.0,
    ultrasonic_dist: 0.0,
  });
  const [sensorCalibration, setSensorCalibration] = useState<SensorCalibration>(
    {
      gyro_calibration: [0, 0, 0],
      accel_calibration: [0, 0, 0],
    }
  );
  const [channels, setChannels] = useState<ElrsChannels>({
    channels: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
  });
  const [hasRunBefore, setHasRunBefore] = useState(false);
  const [chartData, setChartData] = useState({
    labels: [],
    datasets: [
      {
        label: "Filtered Orientation",
        data: [],
        borderColor: "rgba(255, 255, 0, 1)",
        backgroundColor: "rgba(255, 255, 0, 0.2)",
        fill: true,
      },
      {
        label: "Accelerometer Orientation",
        data: [],
        borderColor: "rgba(255, 0, 0, 1)",
        backgroundColor: "rgba(255, 0, 0, 0.2)",
        fill: true,
      },
      {
        label: "Gyroscope Orientation",
        data: [],
        borderColor: "rgba(0, 0, 255, 1)",
        backgroundColor: "rgba(0, 0, 255, 0.2)",
        fill: true,
      },
      {
        label: "Ultrasonic Height",
        data: [],
        borderColor: "rgba(0, 255, 255, 1)",
        backgroundColor: "rgba(0, 255, 255, 0.2)",
        fill: true,
      },
    ],
  });

  const chartOptions = {
    animation: false,
    responsive: true,
    scales: {
      x: {
        title: {
          display: true,
          text: "Time",
        },
      },
      y: {
        title: {
          display: true,
          text: "Degrees (radians) / Height (cm)",
        },
        beginAtZero: true,
      },
    },
  };

  const accelMeshRef = useRef();
  const gyroMeshRef = useRef();
  const orientationMeshRef = useRef();

  useEffect(() => {
    (async () => {
      if (!hasRunBefore) {
        setHasRunBefore(true);
        await invoke("start_usb_loop", {});
      }
    })();
  }, []);

  const start_gyro_calibration = async () => {
    await invoke("start_gyro_calibration", {});
  };

  const max_data_points = 100;
  const [chartChannel, setChartChannel] = useState(0);
  let update_chart = (sensor_data: ImuSensor) => {
    setChartData((prev) => {
      const newLabel = new Date().toLocaleTimeString();

      let gyro_euler = new Euler().setFromQuaternion(
        new Quaternion(
          imuSensors.gyro_orientation[1],
          imuSensors.gyro_orientation[0],
          imuSensors.gyro_orientation[2],
          imuSensors.gyro_orientation[3]
        )
      );
      let accel_euler = new Euler().setFromQuaternion(
        new Quaternion(
          imuSensors.accel_orientation[1],
          imuSensors.accel_orientation[0],
          imuSensors.accel_orientation[2],
          imuSensors.accel_orientation[3]
        )
      );
      let filtered_euler = new Euler().setFromQuaternion(
        new Quaternion(
          imuSensors.orientation[1],
          imuSensors.orientation[0],
          imuSensors.orientation[2],
          imuSensors.orientation[3]
        )
      );
      // let update euler angles
      let euler_angles = {
        gyro: [gyro_euler.x, gyro_euler.y, gyro_euler.z],
        accel: [accel_euler.x, accel_euler.y, accel_euler.z],
        filtered: [filtered_euler.x, filtered_euler.y, filtered_euler.z],
      };
      setEulerImu(euler_angles);

      const updatedLabels = [...prev.labels, newLabel];
      const updatedOrientationData = [
        ...prev.datasets[0].data,
        (euler_angles.filtered[chartChannel] / Math.PI) * 180,
      ];
      const updatedAccelData = [
        ...prev.datasets[1].data,
        (euler_angles.accel[chartChannel] / Math.PI) * 180,
      ];
      const updatedGyroData = [
        ...prev.datasets[2].data,
        (euler_angles.gyro[chartChannel] / Math.PI) * 180,
      ];
      const updatedUltrasonic = [
        ...prev.datasets[3].data,
        sensors.ultrasonic_dist,
      ];

      if (updatedLabels.length > max_data_points) {
        updatedLabels.shift();
        updatedOrientationData.shift();
        updatedAccelData.shift();
        updatedGyroData.shift();
        updatedUltrasonic.shift();
      }

      return {
        labels: updatedLabels,
        datasets: [
          {
            ...prev.datasets[0],
            data: updatedOrientationData,
          },
          {
            ...prev.datasets[1],
            data: updatedAccelData,
          },
          {
            ...prev.datasets[2],
            data: updatedGyroData,
          },
          {
            ...prev.datasets[3],
            data: updatedUltrasonic,
          },
        ],
      };
    });
  };

  useEffect(() => {
    update_chart(imuSensors);
  }, [imuSensors]);

  let [flag, setFlag] = useState(false);
  useEffect(() => {
    let unlisten: any;

    // Register the listener once
    const setupListener = async () => {
      unlisten = await listen<Message>("tc_data", (event) => {
        setFlag((prev) => {
          if (prev == false) {
            return true;
          }

          if ("ImuSensor" in event.payload) {
            setImuSensors(event.payload.ImuSensor);
          } else if ("Sensor" in event.payload) {
            setSensors(event.payload.Sensor);
          } else if ("SensorCalibration" in event.payload) {
            setSensorCalibration(event.payload.SensorCalibration);
          } else if ("State" in event.payload) {
            setState(event.payload.State);
          } else if ("ElrsChannels" in event.payload) {
            setChannels(event.payload.ElrsChannels);
          }

          return false;
        });
      });
    };

    setupListener();

    // Cleanup the listener when the component unmounts
    return () => {
      if (unlisten) {
        unlisten();
      }
    };
  }, []);

  return (
    <main className="flex flex-col h-screen">
      <div className="flex h-[80vh]">
        <div className="w-[25vw]">
          <p>
            Orientation X: {(imuSensors.orientation[0] / Math.PI) * 180}
            <br />
            Orientation Y: {(imuSensors.orientation[1] / Math.PI) * 180}
            <br />
            Orientation Z: {(imuSensors.orientation[2] / Math.PI) * 180}
            <br />
            <br />
            Gyro Orientation X:{" "}
            {(imuSensors.gyro_orientation[0] / Math.PI) * 180}
            <br />
            Gyro Orientation Y:{" "}
            {(imuSensors.gyro_orientation[1] / Math.PI) * 180}
            <br />
            Gyro Orientation Z:{" "}
            {(imuSensors.gyro_orientation[2] / Math.PI) * 180}
            <br />
            <br />
            Accel Orientation X:{" "}
            {(imuSensors.accel_orientation[0] / Math.PI) * 180}
            <br />
            Accel Orientation Y:{" "}
            {(imuSensors.accel_orientation[1] / Math.PI) * 180}
            <br />
            Accel Orientation Z:{" "}
            {(imuSensors.accel_orientation[2] / Math.PI) * 180}
            <br />
            <br />
            Sensor Update Frequency: {state.sensor_update_rate} hz
            <br />
            Estimated Altitude: {sensors.estimated_altitude} m
            <br />
            Ultrasonic Sensor: {sensors.ultrasonic_dist} cm
            <br />
            Channel 1: {JSON.stringify(channels)}
            <br />
            <button onClick={() => start_gyro_calibration()}>
              Start Gyro Calibration
            </button>
            {/* GyroCalib X:{" "}
            {(sensorCalibration.gyro_calibration[0] / Math.PI) * 180}
            <br />
            GyroCalib Y:{" "}
            {(sensorCalibration.gyro_calibration[1] / Math.PI) * 180}
            <br />
            GyroCalib Z:{" "}
            {(sensorCalibration.gyro_calibration[2] / Math.PI) * 180}
            <br />
            <br />
            AccelCalib X: {sensorCalibration.accel_calibration[0]}
            <br />
            AccelCalib Y: {sensorCalibration.accel_calibration[1]}
            <br />
            AccelCalib Z: {sensorCalibration.accel_calibration[2]} */}
          </p>
          <label htmlFor="axis-options">Chart Axis {chartChannel}</label>
          <div id="axis-options" className="flex gap-2">
            <button
              disabled={chartChannel == 0}
              onClick={() => setChartChannel(0)}
            >
              X-Axis
            </button>
            <button
              disabled={chartChannel == 1}
              onClick={() => setChartChannel(1)}
            >
              Y-Axis
            </button>
            <button
              disabled={chartChannel == 2}
              onClick={() => setChartChannel(2)}
            >
              Z-Axis
            </button>
          </div>
        </div>
        <div className="w-[75vw]">
          <Canvas>
            <ambientLight intensity={0.1} />
            <directionalLight color="white" position={[0, 0, 5]} />
            <OrbitControls />
            <Text
              fontSize={0.2}
              color="white"
              anchorX={"center"}
              anchorY={"middle"}
              position={[-2, 1, 0]}
            >
              Accelerometer
            </Text>
            <mesh
              ref={accelMeshRef}
              quaternion={[
                -imuSensors.accel_orientation[0],
                imuSensors.accel_orientation[3],
                imuSensors.accel_orientation[1],
                -imuSensors.accel_orientation[2],
              ]}
              position={[-2, 0, 0]}
            >
              <boxGeometry args={[1.5, 0.1, 1]} />
              <meshStandardMaterial color="red" />
              <Arrow parentMesh={accelMeshRef} />
            </mesh>
            <Text
              fontSize={0.2}
              color="white"
              anchorX={"center"}
              anchorY={"middle"}
              position={[0, 1, 0]}
            >
              Filtered
            </Text>
            <mesh
              ref={accelMeshRef}
              quaternion={[
                -imuSensors.orientation[0],
                imuSensors.orientation[3],
                imuSensors.orientation[1],
                -imuSensors.orientation[2],
              ]}
              position={[0, 0, 0]}
            >
              <boxGeometry args={[1.5, 0.1, 1]} />
              <meshStandardMaterial color="yellow" />
              <Arrow parentMesh={accelMeshRef} />
            </mesh>
            <Text
              fontSize={0.2}
              color="white"
              anchorX={"center"}
              anchorY={"middle"}
              position={[2, 1, 0]}
            >
              Gyroscope
            </Text>
            <mesh
              ref={gyroMeshRef}
              // rotation={[
              //   imuSensors.gyro_orientation[1],
              //   imuSensors.gyro_orientation[2],
              //   imuSensors.gyro_orientation[0],
              // ]}
              quaternion={[
                -imuSensors.gyro_orientation[0],
                imuSensors.gyro_orientation[3],
                imuSensors.gyro_orientation[1],
                -imuSensors.gyro_orientation[2],
              ]}
              position={[2, 0, 0]}
            >
              <boxGeometry args={[1.5, 0.1, 1]} />
              <meshStandardMaterial color="blue" />
              <Arrow parentMesh={accelMeshRef} />
            </mesh>
          </Canvas>
        </div>
      </div>
      <div className="">
        <Line data={chartData} options={chartOptions} />
      </div>
      <div>
        <h1>ELRS Channels</h1>
        <div>{JSON.stringify(channels)}</div>
      </div>
    </main>
  );
}

function Arrow({ parentMesh }: { parentMesh: any }) {
  const arrowRef = useRef();

  // Create arrow geometry
  const arrowLength = 1;
  const arrowHeadSize = 0.2;

  return (
    <group ref={arrowRef}>
      {/* Arrow shaft */}
      <mesh position={[0, -arrowLength / 2, 0]}>
        <cylinderGeometry args={[0.02, 0.02, arrowLength, 12]} />
        <meshBasicMaterial color="green" />
      </mesh>

      {/* Arrow head */}
      <mesh position={[0, -arrowLength, 0]} rotation={[0, 0, Math.PI]}>
        <coneGeometry args={[arrowHeadSize, arrowHeadSize, 12]} />
        <meshBasicMaterial color="green" />
      </mesh>
    </group>
  );
}

export default App;
