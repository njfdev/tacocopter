import { addToast, Button, Slider, Switch } from "@heroui/react";
import { ElrsChannels, TCData } from "../types";
import React, { useEffect, useState } from "react";
import { invoke } from "@tauri-apps/api/core";
import { save } from "@tauri-apps/plugin-dialog";
import { downloadDir } from "@tauri-apps/api/path";
import { FaDownload } from "react-icons/fa6";

export default function BlackboxInterface({ tcData }: { tcData: TCData }) {
  const [isDownloading, setIsDownloading] = useState(false);
  const [path, setPath] = useState("");

  useEffect(() => {
    (async () => {
      if (path == "") {
        setPath((await downloadDir()) + "/tc-blackbox-log.csv");
      }
    })();
  });

  const start_blackbox_download = async () => {
    setIsDownloading(true);
    try {
      let result = await invoke("start_blackbox_download", { path });
      addToast({
        title: "Success!",
        description: `Downloaded ${result} logs to ${path}.`,
        color: "success",
      });
    } catch (e) {
      addToast({
        title: "Error!",
        description: `${e}`,
        color: "danger",
      });
    }
    setIsDownloading(false);
  };

  const set_black_box_enabled = async (enabled: boolean) => {
    await invoke("set_blackbox_enabled", { enabled });
  };

  const open_save_file_dialog = async () => {
    const newPath = await save({
      title: "Select Location of Blackbox File",
      filters: [
        {
          name: "CSV Filter",
          extensions: ["csv"],
        },
      ],
      defaultPath: path,
    });

    if (newPath) {
      setPath(newPath);
    }
  };

  return (
    <div className="mainContentContainer">
      <div className="w-full mb-4 flex gap-3 items-center align-middle">
        <Switch
          isSelected={tcData.blackbox_enabled}
          onValueChange={set_black_box_enabled}
        />
        <h1 className="font-bold text-2xl">Blackbox</h1>
      </div>
      <p className="font-bold">Location to Save Blackbox File</p>
      <Button
        className="mb-4 max-w-lg"
        variant="ghost"
        onPress={() => open_save_file_dialog()}
        endContent={<FaDownload />}
      >
        {path}
      </Button>
      <div className="max-w-xl flex flex-col gap-8">
        <Button
          onPress={() => start_blackbox_download()}
          isLoading={isDownloading}
        >
          Start Download
        </Button>
      </div>
    </div>
  );
}
