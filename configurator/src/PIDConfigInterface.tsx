import { Button, Input, NumberInput, Slider } from "@heroui/react";
import { ElrsChannels, PIDSettings, TCData } from "./types";
import React, { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";

export default function CalibrationInterface({ tcData }: { tcData: TCData }) {
  const [isDraft, setIsDraft] = useState(false);
  const [pitchSettings, setPitchSettings] = useState<[number, number, number]>(
    tcData.pid.pitch
  );
  const [rollSettings, setRollSettings] = useState<[number, number, number]>(
    tcData.pid.roll
  );
  const [yawSettings, setYawSettings] = useState<[number, number, number]>(
    tcData.pid.yaw
  );

  const set_states_to_pid_settings = (settings: PIDSettings) => {
    setPitchSettings(settings.pitch);
    setRollSettings(settings.roll);
    setYawSettings(settings.yaw);
  };

  useEffect(() => {
    if (!isDraft) {
      set_states_to_pid_settings(tcData.pid);
    }
  }, [tcData.pid]);

  const set_pid_settings = async (pid_settings: PIDSettings) => {
    console.log(pid_settings);
    await invoke("set_pid_settings", { pidSettings: pid_settings });
  };

  const updatePidDraft = (
    val: number,
    index: number,
    updateFunc: React.Dispatch<React.SetStateAction<[number, number, number]>>
  ) => {
    setIsDraft(true);
    updateFunc((prev) => {
      let newPid: [number, number, number] = [...prev];

      newPid[index] = val;

      return newPid;
    });
  };

  return (
    <main className="flex flex-col h-screen mainContentContainer pt-4 pl-2">
      <div className="h-full w-full">
        <h1 className="w-full font-bold text-2xl mb-4">PID Configuration</h1>
        <div className="mb-4">
          Scale factor of x100,000. Example: 120 is set to 0.0012 on drone.
        </div>
        <div className="flex flex-col gap-8 w-full">
          <div className="flex flex-col gap-1">
            <h2 className="text-lg font-semibold">Pitch</h2>
            <div className="w-full flex gap-2">
              <NumberInput
                label={`P (${tcData.pid.pitch[0]})`}
                value={pitchSettings[0] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 0, setPitchSettings);
                }}
              />
              <NumberInput
                label={`I (${tcData.pid.pitch[1]})`}
                value={pitchSettings[1] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 1, setPitchSettings);
                }}
              />
              <NumberInput
                label={`D (${tcData.pid.pitch[2]})`}
                value={pitchSettings[2] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 2, setPitchSettings);
                }}
              />
            </div>
          </div>
          <div className="flex flex-col gap-1">
            <h2 className="text-lg font-semibold">Roll</h2>
            <div className="w-full flex gap-2">
              <NumberInput
                label={`P (${tcData.pid.roll[0]})`}
                value={rollSettings[0] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 0, setRollSettings);
                }}
              />
              <NumberInput
                label={`I (${tcData.pid.roll[1]})`}
                value={rollSettings[1] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 1, setRollSettings);
                }}
              />
              <NumberInput
                label={`D (${tcData.pid.roll[2]})`}
                value={rollSettings[2] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 2, setRollSettings);
                }}
              />
            </div>
          </div>
          <div className="flex flex-col gap-1">
            <h2 className="text-lg font-semibold">Yaw</h2>
            <div className="w-full flex gap-2">
              <NumberInput
                label={`P (${tcData.pid.yaw[0]})`}
                value={yawSettings[0] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 0, setYawSettings);
                }}
              />
              <NumberInput
                label={`I (${tcData.pid.yaw[1]})`}
                value={yawSettings[1] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 1, setYawSettings);
                }}
              />
              <NumberInput
                label={`D (${tcData.pid.yaw[2]})`}
                value={yawSettings[2] * 100000}
                step={0.1}
                onValueChange={(val) => {
                  console.log(val / 100000);
                  updatePidDraft(val / 100000, 2, setYawSettings);
                }}
              />
            </div>
          </div>
          <div className="flex gap-2">
            <Button
              color="primary"
              disabled={!isDraft}
              className={`transition-opacity ${
                isDraft
                  ? ""
                  : "opacity-0 hover:cursor-default pointer-events-none"
              }`}
              onPress={() => {
                set_pid_settings({
                  pitch: pitchSettings,
                  roll: rollSettings,
                  yaw: yawSettings,
                });
                setIsDraft(false);
              }}
            >
              Set New Settings
            </Button>
            <Button
              color="default"
              disabled={!isDraft}
              className={`transition-opacity ${
                isDraft
                  ? ""
                  : "opacity-0 hover:cursor-default pointer-events-none"
              }`}
              onPress={() => {
                setIsDraft(false);
                set_states_to_pid_settings(tcData.pid);
              }}
            >
              Cancel
            </Button>
          </div>
        </div>
      </div>
    </main>
  );
}
