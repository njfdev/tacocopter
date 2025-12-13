import { Button, Tab, Tabs } from "@heroui/react";
import MainInterface from "./MainInterface";
import { useEffect, useRef, useState } from "react";
import { Message, TCData } from "./types";
import { listen } from "@tauri-apps/api/event";
import ElrsInterface from "./ElrsInterface";
import LogInterface from "./LogInterface";
import CalibrationInterface from "./config_interface/CalibrationInterface";
import BlackboxInterface from "./BlackboxInterface";
import PIDConfigInterface from "./PIDConfigInterface";
import ConfigurationInterface from "./config_interface/ConfigurationInterface";

function App() {
  const [tcData, setTcData] = useState<TCData>({
    state: {
      uptime: 0,
      imu_process_rate: 0,
      target_update_rate: 0,
      control_loop_update_rate: 0,
      blheli_passthrough: false,
    },
    imuSensors: {
      // gyroscope: [0, 0, 0],
      // accelerometer: [0, 0, 0],
      gyro_orientation: [0, 0, 0, 0],
      accel_orientation: [0, 0, 0, 0],
      orientation: [0, 0, 0, 0],
    },
    sensors: {
      estimated_altitude: 0.0,
      ultrasonic_dist: 0.0,
    },
    sensorCalibration: {
      gyro_calibration: [0, 0, 0],
      accel_calibration: [0, 0, 0],
    },
    channels: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    logs: [],
    pid: {
      pitch: [0, 0, 0],
      roll: [0, 0, 0],
      yaw: [0, 0, 0],
    },
  });
  const logsRef = useRef(tcData.logs);

  useEffect(() => {
    logsRef.current = tcData.logs;
  }, [tcData.logs]);

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
            setTcData((prev) => ({
              ...prev,
              imuSensors: event.payload.ImuSensor,
            }));
          } else if ("Sensor" in event.payload) {
            setTcData((prev) => ({
              ...prev,
              sensors: event.payload.Sensor,
            }));
          } else if ("SensorCalibration" in event.payload) {
            setTcData((prev) => ({
              ...prev,
              sensorCalibration: event.payload.SensorCalibration,
            }));
          } else if ("State" in event.payload) {
            setTcData((prev) => ({
              ...prev,
              state: event.payload.State,
            }));
          } else if ("ElrsChannels" in event.payload) {
            setTcData((prev) => ({
              ...prev,
              channels: event.payload.ElrsChannels,
            }));
          } else if ("PIDSettings" in event.payload) {
            setTcData((prev) => ({
              ...prev,
              pid: event.payload.PIDSettings,
            }));
          } else if ("Blackbox" in event.payload) {
            setTcData((prev) => ({
              ...prev,
              blackbox_enabled: event.payload.Blackbox,
            }));
          }
          // setTcData(() => {
          //   if (prev.log.id != event.payload.Log.id) {
          //     prev.log.text += event.payload.Log.text;

          //     prev.log.id = event.payload.Log.id;
          //   }

          //   return prev;
          // });

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
  useEffect(() => {
    let unlisten: any;

    // Register the listener once
    const setupListener = async () => {
      unlisten = await listen<Message>("tc_log", (event) => {
        // console.log(event.payload);
        if (
          logsRef.current.length == 0 ||
          event.payload[event.payload.length - 1].id !=
            logsRef.current[logsRef.current.length - 1].id
        ) {
          let newLog = logsRef.current.concat(event.payload);

          if (newLog.length > 1000) {
            newLog = newLog.slice(newLog.length - 1001, newLog.length);
          }
          setTcData((prev) => ({ ...prev, logs: newLog }));
        }

        return false;
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
    <main className="flex flex-col h-screen w-screen pl-2 pt-2">
      <Tabs isVertical={true} fullWidth={true}>
        <Tab key="main" title="Main">
          <MainInterface tcData={tcData} />
        </Tab>
        <Tab key="elrs" title="ELRS">
          <ElrsInterface tcData={tcData} />
        </Tab>
        <Tab key="pid" title="PID Config">
          <PIDConfigInterface tcData={tcData} />
        </Tab>
        <Tab key="conf" title="Confiuration">
          <ConfigurationInterface tcData={tcData} />
        </Tab>
        <Tab key="log" title="Logs">
          <LogInterface tcData={tcData} setTcData={setTcData} />
        </Tab>
      </Tabs>
    </main>
  );
}

export default App;
