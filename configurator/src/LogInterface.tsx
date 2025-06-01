import { Button, Slider } from "@heroui/react";
import { ElrsChannels, TCData } from "./types";
import React, { useEffect, useRef, useState } from "react";
import { FaArrowUp } from "react-icons/fa6";

function LogInterface({
  tcData,
  setTcData,
}: {
  tcData: TCData;
  setTcData: React.Dispatch<React.SetStateAction<TCData>>;
}) {
  const logWindowRef = useRef(null);
  const [isAtBottom, setIsAtBottom] = useState(true);

  function checkIsAtBottom() {
    const logWindow = logWindowRef.current;
    if (!logWindow) return true;
    return (
      logWindow.scrollHeight - logWindow.scrollTop - logWindow.clientHeight < 4
    );
  }

  function scrollToBottom() {
    const logWindow = logWindowRef.current;
    if (logWindow) logWindow.scrollTop = logWindow.scrollHeight;
  }

  useEffect(() => {
    setIsAtBottom(checkIsAtBottom());
  }, [logWindowRef?.current?.scrollTop]);

  useEffect(() => {
    const logWindow = logWindowRef.current;
    if (!logWindow) return;

    if (isAtBottom) {
      scrollToBottom();
    }
  }, [tcData.log.text]);

  return (
    <main className="flex flex-col h-screen mainContentContainer">
      <div className="h-full w-full flex flex-col py-4 pl-2">
        <div className="flex justify-center align-middle items-center pb-2">
          <h1 className="w-full text-2xl font-bold">Log</h1>
          <Button
            onPress={() =>
              setTcData((prev) => ({ ...prev, log: { id: -1, text: "" } }))
            }
          >
            Clear
          </Button>
        </div>
        <div className="relative h-full overflow-clip">
          <div
            ref={logWindowRef}
            className="grid grid-rows-8 gap-x-4 grid-flow-col whitespace-pre-line h-full overflow-y-auto font-mono"
          >
            {tcData.log.text}
            <Button
              variant="flat"
              className={`absolute right-6 bottom-1 disabled:!opacity-0 disabled:pointer-events-none`}
              disabled={isAtBottom}
              isIconOnly={true}
              onPress={() => scrollToBottom()}
            >
              <FaArrowUp className="rotate-180" />
            </Button>
          </div>
        </div>
      </div>
    </main>
  );
}

export default LogInterface;
