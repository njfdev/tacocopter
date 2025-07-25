import { Button, Slider } from "@heroui/react";
import { ElrsChannels, TCData } from "./types";
import React, { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";

export default function BlackboxInterface({ tcData }: { tcData: TCData }) {
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

  const start_blackbox_download = async () => {
    setGyroCalibrating(true);
    await invoke("start_blackbox_download", {});
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
        <h1 className="w-full font-bold text-2xl mb-4">Blackbox</h1>
        <div className="max-w-xl flex flex-col gap-8">
          <Button
            onPress={() => start_blackbox_download()}
            isLoading={gyroCalibrating}
          >
            Start Download
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
      </div>
    </main>
  );
}
