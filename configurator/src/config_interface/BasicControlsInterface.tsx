import { addToast, Button, Slider, Switch } from "@heroui/react";
import { ElrsChannels, TCData } from "../types";
import React, { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import { save } from "@tauri-apps/plugin-dialog";
import { downloadDir } from "@tauri-apps/api/path";
import { FaDownload } from "react-icons/fa6";

export default function BasicControlsInterface({ tcData }: { tcData: TCData }) {
  const toggle_blheli_passthrough = async () => {
    await invoke("toggle_blheli_passthrough");
  };
  const reset_to_usb_boot = async () => {
    await invoke("reset_to_usb_boot");
  };

  return (
    <div className="mainContentContainer">
      <h1 className="font-bold text-2xl mb-2">Basic Controls</h1>
      <div className="flex flex-col gap-3">
        <Switch
          isSelected={tcData.state.blheli_passthrough}
          onValueChange={toggle_blheli_passthrough}
        >
          BlHeli FC Passthrough
        </Switch>
        <Button
          className="w-48"
          variant="ghost"
          color="default"
          onPress={reset_to_usb_boot}
        >
          Reset to USB Boot
        </Button>
      </div>
    </div>
  );
}
