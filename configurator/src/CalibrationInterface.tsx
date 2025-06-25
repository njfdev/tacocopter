import { Button, Slider } from "@heroui/react";
import { ElrsChannels, TCData } from "./types";
import React from "react";

export default function CalibrationInterface({ tcData }: { tcData: TCData }) {
  return (
    <main className="flex flex-col h-screen mainContentContainer pt-4 pl-2">
      <div className="h-full w-full">
        <h1 className="w-full font-bold text-2xl mb-4">Calibration</h1>
        <div className="max-w-xl flex flex-col gap-8">
          <div className="flex flex-col gap-1">
            <h2 className="text-lg font-semibold">Gyroscope</h2>
            <p>
              Current Offsets: {tcData.sensorCalibration.gyro_calibration[0]},{" "}
              {tcData.sensorCalibration.gyro_calibration[1]},{" "}
              {tcData.sensorCalibration.gyro_calibration[2]}
            </p>
            <Button>Start Calibration</Button>
          </div>
          <div className="flex flex-col gap-1">
            <h2 className="text-lg font-semibold">Accelerometer</h2>
            <p>
              Current Offsets: {tcData.sensorCalibration.accel_calibration[0]},{" "}
              {tcData.sensorCalibration.accel_calibration[1]},{" "}
              {tcData.sensorCalibration.accel_calibration[2]}
            </p>
            <Button>Start Calibration</Button>
          </div>
        </div>
      </div>
    </main>
  );
}
