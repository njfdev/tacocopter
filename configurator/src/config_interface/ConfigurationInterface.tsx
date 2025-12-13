import BlackboxInterface from "./BlackboxInterface";
import { TCData } from "../types";
import CalibrationInterface from "./CalibrationInterface";
import PassthroughInterface from "./PassthroughInterface";

export default function ConfigurationInterface({ tcData }: { tcData: TCData }) {
  return (
    <main className="flex flex-col h-screen mainContentContainer pt-2 pl-2">
      <div className="h-full w-full flex flex-col gap-12">
        <PassthroughInterface tcData={tcData} />
        <BlackboxInterface tcData={tcData} />
        <CalibrationInterface tcData={tcData} />
      </div>
    </main>
  );
}
