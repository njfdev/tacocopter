import { Button, Slider, Switch } from "@heroui/react";
import { ElrsChannels, TCData } from "./types";
import React, { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";

export default function BlackboxInterface({ tcData }: { tcData: TCData }) {
  const [isDownloading, setIsDownloading] = useState(false);

  const start_blackbox_download = async () => {
    // setIsDownloading(true);
    await invoke("start_blackbox_download", {});
  };

  const set_black_box_enabled = async (enabled: boolean) => {
    await invoke("set_blackbox_enabled", { enabled });
  };

  return (
    <main className="flex flex-col h-screen mainContentContainer pt-2 pl-2">
      <div className="h-full w-full">
        <div className="w-full mb-4 flex gap-3 items-center align-middle">
          <Switch
            isSelected={tcData.blackbox_enabled}
            onValueChange={set_black_box_enabled}
          />
          <h1 className="font-bold text-2xl">Blackbox</h1>
        </div>
        <div className="max-w-xl flex flex-col gap-8">
          <Button
            onPress={() => start_blackbox_download()}
            isLoading={isDownloading}
          >
            Start Download
          </Button>
        </div>
      </div>
    </main>
  );
}
