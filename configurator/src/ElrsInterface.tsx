import { Slider } from "@heroui/react";
import { ElrsChannels, TCData } from "./types";
import React from "react";

function ElrsInterface({ tcData }: { tcData: TCData }) {
  return (
    <main className="flex flex-col h-screen mainContentContainer pl-2 pt-2">
      <div className="h-full w-full">
        <h1 className="w-full font-bold text-xl">ELRS Channels</h1>
        <div className="grid grid-rows-8 gap-x-4 grid-flow-col">
          {tcData.channels.map((val, id) => {
            return (
              <ElrsChannelView
                key={`elrs-channel-${id}`}
                channels={tcData.channels}
                id={id}
              />
            );
          })}
        </div>
      </div>
    </main>
  );
}

function ElrsChannelView({
  channels,
  id,
}: {
  channels?: ElrsChannels;
  id: number;
}) {
  return (
    <Slider
      label={`Channel ${id + 1}`}
      value={channels ? channels[id] : 0}
      className="w-[50wv]"
      hideThumb={true}
      minValue={0}
      maxValue={2000}
      size="lg"
    />
  );
}

export default ElrsInterface;
