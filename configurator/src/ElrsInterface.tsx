import { Slider } from "@heroui/react";
import { TCData } from "./types";

function ElrsInterface({ tcData }: { tcData: TCData }) {
  return (
    <main className="flex flex-col h-screen">
      <div className="h-full">
        <h1>ELRS Channels</h1>
        <div>{JSON.stringify(tcData.channels)}</div>
        <Slider
          aria-label="Channel 1"
          defaultValue={0.5}
          className="w-[50wv]"
          hideThumb={true}
          color="foreground"
          minValue={0}
          maxValue={1}
          size="lg"
        />
      </div>
    </main>
  );
}

export default ElrsInterface;
