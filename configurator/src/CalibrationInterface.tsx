import { Button, Slider } from "@heroui/react";
import { ElrsChannels, TCData } from "./types";
import React, { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";

export default function CalibrationInterface({ tcData }: { tcData: TCData }) {
  const [gyroCalibrating, setGyroCalibrating] = useState(false);
  const [gyroCalibProgress, setGyroCalibProgress] = useState({
    samples: 0,
    seconds_remaining: 10,
  });
  const [calibrationData, setCalibrationData] = useState(
    tcData.sensorCalibration.Data || {
      gyro_calibration: [0, 0, 0],
      accel_calibration: [0, 0, 0],
    }
  );

  const start_gyro_calibration = async () => {
    setGyroCalibrating(true);
    await invoke("start_gyro_calibration", {});
  };

  useEffect(() => {
    if (typeof tcData.sensorCalibration == "string") {
      setGyroCalibrating(false);
    } else if ("Data" in tcData.sensorCalibration) {
      setGyroCalibrating(false);
      setCalibrationData(tcData.sensorCalibration.Data);
    } else if ("GyroProgress" in tcData.sensorCalibration) {
      setGyroCalibrating(true);
      setGyroCalibProgress(tcData.sensorCalibration.GyroProgress);
    }
  }, [tcData.sensorCalibration]);

  return (
    <main className="flex flex-col h-screen mainContentContainer pt-4 pl-2">
      <div className="h-full w-full">
        <h1 className="w-full font-bold text-2xl mb-4">Calibration</h1>
        <div className="max-w-xl flex flex-col gap-8">
          <div className="flex flex-col gap-1">
            <h2 className="text-lg font-semibold">Gyroscope</h2>
            <p>
              Current Offsets: {calibrationData.gyro_calibration[0]},{" "}
              {calibrationData.gyro_calibration[1]},{" "}
              {calibrationData.gyro_calibration[2]}
            </p>
            <Button
              onPress={() => start_gyro_calibration()}
              isLoading={gyroCalibrating}
            >
              Start Calibration
            </Button>
            {gyroCalibrating && (
              <Slider
                label={`
              Samples: ${gyroCalibProgress.samples}`}
                value={10 - gyroCalibProgress.seconds_remaining}
                minValue={0}
                maxValue={10}
                step={0.01}
                hideValue={true}
                hideThumb={true}
              ></Slider>
            )}
          </div>
          <div className="flex flex-col gap-1">
            <h2 className="text-lg font-semibold">Accelerometer</h2>
            <p>
              Current Offsets: {calibrationData.accel_calibration[0]},{" "}
              {calibrationData.accel_calibration[1]},{" "}
              {calibrationData.accel_calibration[2]}
            </p>
            <Button>Start Calibration</Button>
          </div>
        </div>
      </div>
    </main>
  );
}
