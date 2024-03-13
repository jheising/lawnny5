import { useROS } from "../hooks/useROS";
import { useEffect, useRef, useState } from "react";
import { Message, Topic } from "roslib";
import { JoystickPad, JoystickPosition } from "./JoystickPad";
import { PowerButton } from "./PowerButton";
import { DotMatrixScreen } from "./DotMatrixScreen";
import { Button, Chip } from "ui-neumorphism";
import { useGamepad } from "../hooks/useGamepad";

const DEFAULT_ROS_PORT = 9090;
const JOYSTICK_SIZE = 200;

export function Controller() {
    const [rosURL, setROSURL] = useState<string>();
    const [powerOn, setPowerOn] = useState(false);
    const [isESTOP, setIsESTOP] = useState(false);
    const [displayMessage, setDisplayMessage] = useState<string>();
    const [displayMessageIsError, setDisplayMessageIsError] = useState(false);
    const joystickPublisher = useRef<Topic>();
    const eStopPublisher = useRef<Topic>();
    const gamepad = useGamepad({
        onJoystickPosition: handleJoystickPosition
    });

    const ros = useROS({
        rosURL: rosURL,
        enabled: powerOn
    });

    useEffect(() => {
        // Automatically connect to the ROS server running on this device when the page loads
        setROSURL(`ws://${document.location.hostname}:${DEFAULT_ROS_PORT}`);

        window.scrollTo(0, 1);
    }, []);

    useEffect(() => {

        setDisplayMessageIsError(false);

        if (!powerOn) {
            setDisplayMessage(undefined);
        } else if (isESTOP) {
            setDisplayMessage("ESTOP — MUST RESET");
            setDisplayMessageIsError(true);
        } else if (ros.lastError) {
            setDisplayMessage("CONNECTION ERROR");
            setDisplayMessageIsError(true);
        } else if (ros.isConnected) {
            setDisplayMessage("CONNECTED");
        } else {
            setDisplayMessage("DISCONNECTED");
        }
    }, [ros.isConnected, ros.lastError, isESTOP, powerOn]);

    useEffect(() => {
        if (ros.isConnected && ros.ros) {
            joystickPublisher.current = new Topic({
                ros: ros.ros,
                name: "joy",
                messageType: "sensor_msgs/Joy"
            });

            eStopPublisher.current = new Topic({
                ros: ros.ros,
                name: "cmd_estop",
                messageType: "std_msgs/Bool"
            });

        } else {
            joystickPublisher.current = undefined;
            eStopPublisher.current = undefined;
        }
    }, [ros.isConnected]);

    function handleEStopButton() {
        if (eStopPublisher.current) {
            eStopPublisher.current.publish(new Message({
                data: true
            }));
            setIsESTOP(true);
        }
    }

    function handleJoystickPosition(position?: JoystickPosition) {
        if (position && joystickPublisher.current) {
            joystickPublisher.current.publish(new Message({
                axes: [position.xPercent, position.yPercent],
                buttons: []
            }));
        }
    }

    function togglePowerOn() {
        setPowerOn(!powerOn);
    }

    return <div className="relative h-[calc(100dvh)] max-h-[480px] w-full bg-[#444444] flex flex-row">
        <div className="flex-1 flex flex-col">
            <div className="p-4 pr-0 flex items-center gap-4">
                <PowerButton on={powerOn} onClick={togglePowerOn} />
                <div className="flex-1"><DotMatrixScreen text={displayMessage} isError={displayMessageIsError} /></div>
            </div>
            <div className="flex-1"></div>
            <div className="p-4">
                {gamepad.connected && <Chip dark key="4" type="info" className="ma-3">
                    <svg
                        viewBox="0 0 24 24"
                        fill="currentColor"
                        className="h-6 mr-2"
                    >
                        <path
                            d="M6 9h2v2h2v2H8v2H6v-2H4v-2h2V9m12.5 0a1.5 1.5 0 011.5 1.5 1.5 1.5 0 01-1.5 1.5 1.5 1.5 0 01-1.5-1.5A1.5 1.5 0 0118.5 9m-3 3a1.5 1.5 0 011.5 1.5 1.5 1.5 0 01-1.5 1.5 1.5 1.5 0 01-1.5-1.5 1.5 1.5 0 011.5-1.5M17 5a7 7 0 017 7 7 7 0 01-7 7c-1.96 0-3.73-.8-5-2.1A6.96 6.96 0 017 19a7 7 0 01-7-7 7 7 0 017-7h10M7 7a5 5 0 00-5 5 5 5 0 005 5c1.64 0 3.09-.79 4-2h2c.91 1.21 2.36 2 4 2a5 5 0 005-5 5 5 0 00-5-5H7z" />
                    </svg>
                    Gamepad
                </Chip>}
            </div>
            {/*<div className="w-4 h-4 border-sky-200 shadow-[0_0_2px_#fff,inset_0_0_2px_#fff,0_0_5px_#08f,0_0_15px_#08f,0_0_30px_#08f] bg-sky-200 rounded-full"/>*/}
            {/*{!!ros.ros && <VideoDisplay ros={ros.ros} className="object-contain w-full h-full rounded-xl"/>}*/}
            {/*<button className="bg-red-500 hover:bg-red-400 w-60 h-60 text-white text-4xl font-bold py-2 px-4 border-b-4 border-red-700 hover:border-red-500 rounded-full"*/}
            {/*        onClick={handleEStopButton}>*/}
            {/*    STOP*/}
            {/*</button>*/}
        </div>
        <div className="h-full w-1/3 p-4 flex flex-col gap-2">
            <div className="flex-1 relative">
                <JoystickPad size={JOYSTICK_SIZE} onJoystickPosition={handleJoystickPosition} />
            </div>
            <Button dark size="large" className="!h-1/5 relative overflow-hidden" onClick={handleEStopButton}>
                <div className="absolute left-2 rounded top-2 right-2 bottom-2 font-light text-rose-200 emergency-stripes flex items-center justify-center">
                    <svg
                        fill="currentColor"
                        viewBox="0 0 16 16"
                        className="w-8 h-8"
                    >
                        <path d="M16 8A8 8 0 110 8a8 8 0 0116 0zM6.5 5A1.5 1.5 0 005 6.5v3A1.5 1.5 0 006.5 11h3A1.5 1.5 0 0011 9.5v-3A1.5 1.5 0 009.5 5h-3z" />
                    </svg>
                </div>
            </Button>
        </div>
    </div>;
}