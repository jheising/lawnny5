import { useROS } from "../hooks/useROS";
import { useEffect, useRef } from "react";
import { Message, Topic } from "roslib";
import { JoystickPad, JoystickPosition } from "./JoystickPad";
import { VideoDisplay } from "./VideoDisplay";

const rosServer = "ws://lawnny5.local:9090";
const JOYSTICK_SIZE = 200;
const HALF_JOYSTICK_SIZE = JOYSTICK_SIZE / 2;

export function Controller() {
    const ros = useROS({
        rosURL: rosServer,
        enabled: true
    });
    const joystickPublisher = useRef<Topic>();
    const eStopPublisher = useRef<Topic>();

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

    return <div className="relative h-screen max-h-96 w-full bg-gray-800 flex flex-row shadow-xl drop-shadow-xl">
        <div className="flex-1">
            {!!ros.ros && <VideoDisplay ros={ros.ros} className="object-contain w-full h-full rounded-xl"/>}
            {/*<button className="bg-red-500 hover:bg-red-400 w-60 h-60 text-white text-4xl font-bold py-2 px-4 border-b-4 border-red-700 hover:border-red-500 rounded-full"*/}
            {/*        onClick={handleEStopButton}>*/}
            {/*    STOP*/}
            {/*</button>*/}
        </div>
        <div className="h-full w-1/3 p-4">
            <div className="w-full h-full bg-gray-700 rounded-xl border border-gray-900">
                <JoystickPad size={JOYSTICK_SIZE} onJoystickPosition={handleJoystickPosition} />
            </div>
        </div>
    </div>;
}