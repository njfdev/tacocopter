import { Button, Tab, Tabs } from "@heroui/react";
import MainInterface from "./MainInterface";
import { useEffect, useState } from "react";
import { Message, TCData } from "./types";
import { listen } from "@tauri-apps/api/event";
import ElrsInterface from "./ElrsInterface";
import LogInterface from "./LogInterface";

function App() {
  const [tcData, setTcData] = useState<TCData>({
    state: {
      sensor_update_rate: 0,
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
    log: {
      id: -1,
      text: "",
    },
  });

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
          } else if ("Log" in event.payload) {
            setTcData((prev) => {
              if (prev.log.id != event.payload.Log.id) {
                prev.log.text += event.payload.Log.text;

                prev.log.id = event.payload.Log.id;
              }

              return prev;
            });
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
    <main className="flex flex-col h-screen w-screen">
      <Tabs isVertical={true} fullWidth={true}>
        <Tab key="main" title="Main">
          <MainInterface tcData={tcData} />
        </Tab>
        <Tab key="elrs" title="ELRS">
          <ElrsInterface tcData={tcData} />
        </Tab>
        <Tab key="log" title="Logs">
          <LogInterface tcData={tcData} setTcData={setTcData} />
        </Tab>
      </Tabs>
    </main>
  );
}

export default App;
