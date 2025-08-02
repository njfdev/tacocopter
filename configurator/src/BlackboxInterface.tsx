import { Button, Slider } from "@heroui/react";
import { ElrsChannels, TCData } from "./types";
import React, { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";

export default function BlackboxInterface({ tcData }: { tcData: TCData }) {
  const [isDownloading, setIsDownloading] = useState(false);

  const start_blackbox_download = async () => {
    // setIsDownloading(true);
    await invoke("start_blackbox_download", {});
  };

  return (
    <main className="flex flex-col h-screen mainContentContainer pt-4 pl-2">
      <div className="h-full w-full">
        <h1 className="w-full font-bold text-2xl mb-4">Blackbox</h1>
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
