import {useROS} from "../hooks/useROS";
import {useEffect, useRef, useState} from "react";
import {Message, Topic} from "roslib";
import {JoystickPad, JoystickPosition} from "./JoystickPad";
import {VideoDisplay} from "./VideoDisplay";
import {PowerButton} from "./PowerButton";
import {DotMatrixScreen} from "./DotMatrixScreen";

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

    const ros = useROS({
        rosURL: rosURL,
        enabled: powerOn
    });

    useEffect(() => {
        // Automatically connect to the ROS server running on this device when the page loads
        setROSURL(`ws://${document.location.hostname}:${DEFAULT_ROS_PORT}`)
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

    return <div className="relative h-screen max-h-[480px] w-full bg-gray-800 flex flex-row">
        <div className="flex-1 flex flex-col">
            <div className="p-4 pr-0 flex items-center gap-4">
                <PowerButton on={powerOn} onClick={togglePowerOn}/>
                <div className="flex-1"><DotMatrixScreen text={displayMessage} isError={displayMessageIsError}/></div>
            </div>
            <div className="flex-1"></div>
            {/*<div className="w-4 h-4 border-sky-200 shadow-[0_0_2px_#fff,inset_0_0_2px_#fff,0_0_5px_#08f,0_0_15px_#08f,0_0_30px_#08f] bg-sky-200 rounded-full"/>*/}
            {/*{!!ros.ros && <VideoDisplay ros={ros.ros} className="object-contain w-full h-full rounded-xl"/>}*/}
            {/*<button className="bg-red-500 hover:bg-red-400 w-60 h-60 text-white text-4xl font-bold py-2 px-4 border-b-4 border-red-700 hover:border-red-500 rounded-full"*/}
            {/*        onClick={handleEStopButton}>*/}
            {/*    STOP*/}
            {/*</button>*/}
        </div>
        <div className="h-full w-1/3 p-4">
            <div className="w-full h-full bg-gray-700 rounded-xl border border-gray-900 relative overflow-hidden">
                <JoystickPad size={JOYSTICK_SIZE} onJoystickPosition={handleJoystickPosition}/>
                <button className="absolute bottom-0 left-0 right-0 h-1/5 bg-rose-700 emergency-stripes cursor-pointer text-2xl font-bold" onClick={handleEStopButton}>
                    STOP
                </button>
            </div>
        </div>
    </div>;
}