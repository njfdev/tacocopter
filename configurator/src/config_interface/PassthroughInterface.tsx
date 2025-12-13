import { addToast, Button, Slider, Switch } from "@heroui/react";
import { ElrsChannels, TCData } from "../types";
import React, { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import { save } from "@tauri-apps/plugin-dialog";
import { downloadDir } from "@tauri-apps/api/path";
import { FaDownload } from "react-icons/fa6";

export default function PassthroughInterface({ tcData }: { tcData: TCData }) {
  const toggle_blheli_passthrough = async () => {
    await invoke("toggle_blheli_passthrough");
  };

  return (
    <div className="mainContentContainer">
      <div className="w-full flex gap-3 items-center align-middle">
        <Switch
          isSelected={tcData.state.blheli_passthrough}
          onValueChange={toggle_blheli_passthrough}
        />
        <h1 className="font-bold text-2xl">BlHeli FC Passthrough</h1>
      </div>
    </div>
  );
}
