import { Button, Slider, Switch } from "@heroui/react";
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
  const [showLogLevel, setShowLogLevel] = useState(true);

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
  }, [tcData.logs]);

  return (
    <main className="flex flex-col h-screen mainContentContainer">
      <div className="h-full w-full flex flex-col py-4 pl-2">
        <div className="flex justify-between align-middle items-center pb-2">
          <div className="flex flex-col gap-2">
            <h1 className="w-full text-2xl font-bold">Log</h1>
            <Switch
              size="sm"
              isSelected={showLogLevel}
              onValueChange={setShowLogLevel}
            >
              Show Log Level?
            </Switch>
          </div>
          <Button onPress={() => setTcData((prev) => ({ ...prev, logs: [] }))}>
            Clear
          </Button>
        </div>
        <div className="relative h-full overflow-clip">
          <div
            ref={logWindowRef}
            className="whitespace-pre-wrap h-full overflow-y-auto font-mono"
          >
            {tcData.logs.map((logLine) => {
              let logColor =
                logLine.level == "ERROR"
                  ? "text-red-600"
                  : logLine.level == "WARN"
                  ? "text-yellow-500"
                  : "text-foreground";
              return (
                <p>
                  {showLogLevel && (
                    <span className={logColor}>
                      [{logLine.level}]{" ".repeat(6 - logLine.level.length)}
                    </span>
                  )}
                  <span className={`${showLogLevel ? "" : logColor}`}>
                    {logLine.text}
                  </span>
                </p>
              );
            })}
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
