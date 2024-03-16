import { useROS } from "../hooks/useROS";
import { ChangeEvent, useEffect, useRef, useState } from "react";
import { Message, Topic } from "roslib";
import { JoystickPad, JoystickPosition } from "./JoystickPad";
import { PowerButton } from "./PowerButton";
import { DotMatrixScreen } from "./DotMatrixScreen";
import { Button, Card, Chip, Switch } from "ui-neumorphism";
import { useGamepad } from "../hooks/useGamepad";
import { MathUtils } from "../utils/MathUtils";

const DEFAULT_ROS_PORT = 9090;
const JOYSTICK_SIZE = 200;

export interface ControllerSettings {
    direction: 1 | 2;
    xExpo: number;
    yExpo: number;
}

export function Controller() {
    const [rosURL, setROSURL] = useState<string>();
    const [powerOn, setPowerOn] = useState(false);
    const [isESTOP, setIsESTOP] = useState(false);
    const [displayMessage, setDisplayMessage] = useState<string>();
    const [displayMessageIsError, setDisplayMessageIsError] = useState(false);
    const joystickPublisher = useRef<Topic>();
    const eStopPublisher = useRef<Topic>();
    const [joystickPosition, setJoystickPosition] = useState<JoystickPosition>();
    const [settings, setSettings] = useState<ControllerSettings>({
        direction: 1,
        xExpo: 0,
        yExpo: 0
    });

    const gamepad = useGamepad({
        onJoystickPosition: handleJoystickPositionChanged
    });

    const ros = useROS({
        rosURL: rosURL,
        enabled: powerOn
    });

    useEffect(() => {
        // Automatically connect to the ROS server running on this device when the page loads
        setROSURL(`ws://${document.location.hostname}:${DEFAULT_ROS_PORT}`);
        window.scrollTo(0, 1);

        loadSettings();
    }, []);

    function loadSettings() {
        const savedSettings: ControllerSettings = JSON.parse(localStorage.getItem("settings") ?? "{}");
        updateSettings(savedSettings);
    }

    function updateSettings(newSettings: Partial<ControllerSettings>) {
        const fullSettings = {
            ...settings,
            ...newSettings
        };

        setSettings(fullSettings);
        localStorage.setItem("settings", JSON.stringify(fullSettings));
    }

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

    function handleJoystickPositionChanged(position?: JoystickPosition) {
        const xPercentExpo = MathUtils.expoFunction(position?.xPercent ?? 0, settings.xExpo);
        const yPercentExpo = MathUtils.expoFunction(position?.yPercent ?? 0, settings.yExpo);

        // Send joystick position to ROS
        if (joystickPublisher.current) {
            joystickPublisher.current.publish(new Message({
                axes: [xPercentExpo, settings.direction === 2 ? -yPercentExpo : yPercentExpo],
                buttons: []
            }));
        }

        // Update the UI
        setJoystickPosition({
            xPercent: xPercentExpo,
            yPercent: yPercentExpo
        });
    }

    function togglePowerOn() {
        setPowerOn(!powerOn);
    }

    function handleDirectionChange({ checked }: { checked: boolean }) {
        updateSettings({
            direction: checked ? 2 : 1
        });
    }

    function handleXExpoChange(event: ChangeEvent<HTMLInputElement>) {
        updateSettings({
            xExpo: event.target.valueAsNumber
        });
    }

    function handleYExpoChange(event: ChangeEvent<HTMLInputElement>) {
        updateSettings({
            yExpo: event.target.valueAsNumber
        });
    }

    const joystickX = (joystickPosition?.xPercent ?? 0);
    const joystickY = (joystickPosition?.yPercent ?? 0);

    return <div className="relative h-[calc(100dvh)] max-h-[480px] w-full bg-[#444444] flex flex-row">
        <div className="flex-1 flex flex-col">
            <div className="p-4 pr-0 flex items-center gap-4">
                <PowerButton on={powerOn} onClick={togglePowerOn} />
                <div className="flex-1"><DotMatrixScreen text={displayMessage} isError={displayMessageIsError} /></div>
            </div>
            <div className="flex-1 px-4 flex flex-col justify-center gap-4">
                <div><Switch dark bordered size="large" label={settings.direction === 2 ? "Reverse Direction" : "Forward Direction"} checked={settings.direction === 2} onChange={handleDirectionChange} /></div>
                <div className="flex items-center gap-4"><input type="range" min="0" max="1" value={settings.xExpo} className="w-1/2" step={0.05} onChange={handleXExpoChange} /> X Expo</div>
                <div className="flex items-center gap-4"><input type="range" min="0" max="1" value={settings.yExpo} className="w-1/2" step={0.05} onChange={handleYExpoChange} /> Y Expo</div>
            </div>
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
        </div>
        <div className="h-full w-1/3 p-4 flex flex-col gap-2">
            <div className="flex-1 relative flex flex-col gap-2">
                <div className="flex-1 flex flex-row gap-2">
                    <div className="flex-1"><JoystickPad size={JOYSTICK_SIZE} onJoystickPosition={handleJoystickPositionChanged} /></div>
                    <Card dark inset className="!h-full !w-1 relative overflow-hidden">
                        <div className="absolute w-full bg-sky-400 rounded-full" style={{
                            height: `${Math.abs(joystickY) * 50}%`,
                            bottom: joystickY > 0 ? "50%" : undefined,
                            top: joystickY > 0 ? undefined : "50%"
                        }} />
                    </Card>
                </div>
                <Card dark inset className="w-full h-1 relative overflow-hidden mr-4">
                    <div className="absolute h-full bg-sky-400 rounded-full" style={{
                        width: `${Math.abs(joystickX) * 50}%`,
                        left: joystickX > 0 ? "50%" : undefined,
                        right: joystickX > 0 ? undefined : "50%"
                    }} />
                </Card>
            </div>
            <Button dark bordered size="large" className="!h-1/5 relative overflow-hidden" onClick={handleEStopButton}>
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