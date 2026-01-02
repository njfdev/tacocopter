import { useEffect, useRef, useState } from "react";
import reactLogo from "./assets/react.svg";
import { invoke } from "@tauri-apps/api/core";
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
import { Camera, Euler, PerspectiveCamera, Quaternion } from "three";
import { ImuSensor, TCData } from "./types";
import { Button, ButtonGroup } from "@heroui/react";

function MainInterface({ tcData }: { tcData: TCData }) {
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

  // const [eulerImu, setEulerImu] = useState({
  //   gyro: [0.0, 0.0, 0.0],
  //   accel: [0.0, 0.0, 0.0],
  //   filtered: [0.0, 0.0, 0.0],
  // });
  const [hasRunBefore, setHasRunBefore] = useState(false);
  const [chartData, setChartData] = useState({
    labels: [],
    datasets: [
      // {
      //   label: "Filtered Orientation",
      //   data: [],
      //   borderColor: "rgba(255, 255, 0, 1)",
      //   backgroundColor: "rgba(255, 255, 0, 0.2)",
      //   fill: true,
      // },
      {
        label: "Accelerometer",
        data: [],
        borderColor: "rgba(255, 0, 0, 1)",
        backgroundColor: "rgba(255, 0, 0, 0.2)",
        fill: true,
      },
      {
        label: "Gyroscope",
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
          text: "Turn Rate (degrees/s) / Height (cm)",
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

  const max_data_points = 100;
  const [chartChannel, setChartChannel] = useState(0);
  let update_chart = (sensor_data: ImuSensor) => {
    setChartData((prev) => {
      const newLabel = new Date().toLocaleTimeString();

      // let gyro_euler = new Euler().setFromQuaternion(
      //   new Quaternion(
      //     tcData.imuSensors.gyro_orientation[1],
      //     tcData.imuSensors.gyro_orientation[0],
      //     tcData.imuSensors.gyro_orientation[2],
      //     tcData.imuSensors.gyro_orientation[3]
      //   )
      // );
      // let accel_euler = new Euler().setFromQuaternion(
      //   new Quaternion(
      //     tcData.imuSensors.accel_orientation[1],
      //     tcData.imuSensors.accel_orientation[0],
      //     tcData.imuSensors.accel_orientation[2],
      //     tcData.imuSensors.accel_orientation[3]
      //   )
      // );
      // let filtered_euler = new Euler().setFromQuaternion(
      //   new Quaternion(
      //     tcData.imuSensors.orientation[1],
      //     tcData.imuSensors.orientation[0],
      //     tcData.imuSensors.orientation[2],
      //     tcData.imuSensors.orientation[3]
      //   )
      // );
      // // let update euler angles
      // let euler_angles = {
      //   gyro: [gyro_euler.x, gyro_euler.y, gyro_euler.z],
      //   accel: [accel_euler.x, accel_euler.y, accel_euler.z],
      //   filtered: [filtered_euler.x, filtered_euler.y, filtered_euler.z],
      // };
      // setEulerImu(euler_angles);

      const updatedLabels = [...prev.labels, newLabel];
      // const updatedOrientationData = [
      //   ...prev.datasets[0].data,
      //   (euler_angles.filtered[chartChannel] / Math.PI) * 180,
      // ];
      const updatedAccelData = [
        ...prev.datasets[0].data,
        tcData.imuSensors?.accelerometer?.[chartChannel] || 0,
      ];
      const updatedGyroData = [
        ...prev.datasets[1].data,
        ((tcData.imuSensors?.gyroscope?.[chartChannel] || 0) / Math.PI) * 180,
      ];
      const updatedUltrasonic = [
        ...prev.datasets[2].data,
        tcData.sensors.ultrasonic_dist,
      ];

      if (updatedLabels.length > max_data_points) {
        updatedLabels.shift();
        // updatedOrientationData.shift();
        updatedAccelData.shift();
        updatedGyroData.shift();
        updatedUltrasonic.shift();
      }

      return {
        labels: updatedLabels,
        datasets: [
          // {
          //   ...prev.datasets[0],
          //   data: updatedOrientationData,
          // },
          {
            ...prev.datasets[0],
            data: updatedAccelData,
          },
          {
            ...prev.datasets[1],
            data: updatedGyroData,
          },
          {
            ...prev.datasets[2],
            data: updatedUltrasonic,
          },
        ],
      };
    });
  };

  useEffect(() => {
    update_chart(tcData.imuSensors);
  }, [tcData.imuSensors]);

  useEffect(() => {
    if (
      Math.abs(
        1 -
          tcData.state.realtime_loop_update_rate /
            tcData.state.target_update_rate
      ) > 0.05
    ) {
      console.warn(
        `Logs indicate the realtime loop frequency (${tcData.state.realtime_loop_update_rate}) has deviated from the target frequency (${tcData.state.target_update_rate}) by over 5%.`
      );
    }
  }, [tcData.state]);

  return (
    <main className="flex flex-col h-screen">
      <div className="flex h-[30vh] w-[80vw]">
        <div className="w-[30%]">
          <p>
            <b>Uptime:</b> {formatTime(tcData.state.uptime)}
            <br />
            Target Update Frequency: {tcData.state.target_update_rate} hz
            <br />
            <span
              className={`${
                Math.abs(
                  1 -
                    tcData.state.realtime_loop_update_rate /
                      tcData.state.target_update_rate
                ) > 0.05
                  ? "text-red-500"
                  : "text-foreground"
              }`}
            >
              Realtime Loop Update Frequency:{" "}
              {tcData.state.realtime_loop_update_rate} hz
            </span>
            <br />
            Estimated Altitude: {tcData.sensors.estimated_altitude} m
            <br />
            Ultrasonic Sensor: {tcData.sensors.ultrasonic_dist} cm
            <br />
            {/* <Button variant="faded" onPress={() => start_gyro_calibration()}>
              Start Gyro Calibration
            </Button> */}
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
        </div>
        <div className="w-[75%] h-[30vh] relative">
          <div className="absolute w-full h-full flex items-center align-middle justify-center z-10">
            <p className="max-w-[32rem] text-center mx-4 font-bold dark:text-neutral-400 text-gray-700">
              This view will be re-enabled when position-holding logic is
              implemented (otherwise it provides no useful information).
            </p>
          </div>
          <div className="w-full h-full opacity-30 pointer-events-none">
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
                // quaternion={[
                //   -tcData.imuSensors.accel_orientation[0],
                //   tcData.imuSensors.accel_orientation[3],
                //   tcData.imuSensors.accel_orientation[1],
                //   -tcData.imuSensors.accel_orientation[2],
                // ]}
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
                // quaternion={[
                //   -tcData.imuSensors.orientation[0],
                //   tcData.imuSensors.orientation[3],
                //   tcData.imuSensors.orientation[1],
                //   -tcData.imuSensors.orientation[2],
                // ]}
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
                //   tcData.imuSensors.gyro_orientation[1],
                //   tcData.imuSensors.gyro_orientation[2],
                //   tcData.imuSensors.gyro_orientation[0],
                // ]}
                // quaternion={[
                //   -tcData.imuSensors.gyro_orientation[0],
                //   tcData.imuSensors.gyro_orientation[3],
                //   tcData.imuSensors.gyro_orientation[1],
                //   -tcData.imuSensors.gyro_orientation[2],
                // ]}
                position={[2, 0, 0]}
              >
                <boxGeometry args={[1.5, 0.1, 1]} />
                <meshStandardMaterial color="blue" />
                <Arrow parentMesh={accelMeshRef} />
              </mesh>
            </Canvas>
          </div>
        </div>
      </div>
      <div className="flex justify-between mr-4">
        {/* <p className="grow basis-1">
          <b>Orientation X:</b>{" "}
          {(tcData.imuSensors.orientation[0] / Math.PI) * 180}
          <br />
          <b>Orientation Y:</b>{" "}
          {(tcData.imuSensors.orientation[1] / Math.PI) * 180}
          <br />
          <b>Orientation Z:</b>{" "}
          {(tcData.imuSensors.orientation[2] / Math.PI) * 180}
        </p> */}
        <p className="grow basis-1">
          <b>Gyro Orientation X:</b>{" "}
          {((tcData.imuSensors?.gyroscope?.[0] || 0) / Math.PI) * 180}
          <br />
          <b>Gyro Orientation Y:</b>{" "}
          {((tcData.imuSensors?.gyroscope?.[1] || 0) / Math.PI) * 180}
          <br />
          <b>Gyro Orientation Z:</b>{" "}
          {((tcData.imuSensors?.gyroscope?.[2] || 0) / Math.PI) * 180}
        </p>
        <p className="grow basis-1">
          <b>Accel X:</b> {tcData.imuSensors?.accelerometer?.[0] || 0}
          <br />
          <b>Accel Y:</b> {tcData.imuSensors?.accelerometer?.[1] || 0}
          <br />
          <b>Accel Z:</b> {tcData.imuSensors?.accelerometer?.[2] || 0}
        </p>
      </div>
      <div className="min-h-[55vh] mt-4">
        <div id="axis-options" className="flex gap-2">
          <ButtonGroup>
            <Button
              disabled={chartChannel == 0}
              color={chartChannel == 0 ? "primary" : "default"}
              onPress={() => setChartChannel(0)}
            >
              X-Axis
            </Button>
            <Button
              disabled={chartChannel == 1}
              color={chartChannel == 1 ? "primary" : "default"}
              onPress={() => setChartChannel(1)}
            >
              Y-Axis
            </Button>
            <Button
              disabled={chartChannel == 2}
              color={chartChannel == 2 ? "primary" : "default"}
              onPress={() => setChartChannel(2)}
            >
              Z-Axis
            </Button>
          </ButtonGroup>
        </div>
        <div className="h-[55vh]">
          <Line data={chartData} options={chartOptions} />
        </div>
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

function formatTime(total_seconds: number): string {
  let time_string = "";

  let seconds = String(total_seconds % 60).padStart(2, "0");
  let total_minutes = Math.floor(total_seconds / 60);
  let minutes = String(total_minutes % 60).padStart(2, "0");
  let hours = String(Math.floor(total_minutes / 60)).padStart(2, "0");

  if (total_minutes >= 60) {
    time_string += hours + ":";
  }

  if (total_minutes > 0) {
    time_string += minutes + ":";
  }

  time_string += seconds;

  return time_string;
}

export default MainInterface;
