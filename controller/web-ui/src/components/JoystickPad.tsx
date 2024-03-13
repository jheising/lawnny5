import Nipple, { EventData, JoystickOutputData } from "react-nipplejs";
import { useEffect, useRef } from "react";
import { Card } from "ui-neumorphism";
import { useStateCallback } from "../hooks/useStateCallback";

export interface JoystickPosition {
    xPercent: number;
    yPercent: number;
}

export interface JoystickPadProps {
    size: number;
    onJoystickPosition?: (position?: JoystickPosition) => void;
    outputInterval?: number;
    deadZone?: number;
}

export function JoystickPad(props: JoystickPadProps) {
    const joystickContainerRef = useRef<HTMLDivElement>(null);
    const isJoystickActive = useRef(false);

    const joystickState = useStateCallback({
        onValueChanged: props.onJoystickPosition,
        repeatIntervalInMS: props.outputInterval ?? 150,
        shouldRepeat: (value) => {
            if (!isJoystickActive.current || !value) {
                return false;
            }

            const deadZone = props.deadZone ?? 0.009;

            if (Math.abs(value.xPercent) > deadZone || Math.abs(value.yPercent) > deadZone) {
                return true;
            }

            return false;
        }
    });

    function handleJoystickStart(event: EventData, data: JoystickOutputData) {
        isJoystickActive.current = true;
    }

    function handleJoystickEnd(event: EventData, data: JoystickOutputData) {
        isJoystickActive.current = false;
        joystickState.updateValue({
            xPercent: 0,
            yPercent: 0
        });
    }

    function handleJoystickMove(event: EventData, data: JoystickOutputData) {
        const posX = data.distance * Math.cos(data.angle.radian);
        const posY = data.distance * Math.sin(data.angle.radian);
        const halfJoystickSize = props.size / 2;

        const xPercent = posX / halfJoystickSize;
        const yPercent = posY / halfJoystickSize;

        joystickState.updateValue({
            xPercent: xPercent,
            yPercent: yPercent
        });
    }

    return <Card dark inset className="w-full h-full relative" bordered ref={joystickContainerRef}>
        <div className="absolute size-full flex justify-center items-center">
            <svg
                fill="currentColor"
                viewBox="0 0 16 16"
                className="h-10 w-10 opacity-10 pointer-events-none"
            >
                <path
                    fillRule="evenodd"
                    d="M7.646.146a.5.5 0 01.708 0l2 2a.5.5 0 01-.708.708L8.5 1.707V5.5a.5.5 0 01-1 0V1.707L6.354 2.854a.5.5 0 11-.708-.708l2-2zM8 10a.5.5 0 01.5.5v3.793l1.146-1.147a.5.5 0 01.708.708l-2 2a.5.5 0 01-.708 0l-2-2a.5.5 0 01.708-.708L7.5 14.293V10.5A.5.5 0 018 10zM.146 8.354a.5.5 0 010-.708l2-2a.5.5 0 11.708.708L1.707 7.5H5.5a.5.5 0 010 1H1.707l1.147 1.146a.5.5 0 01-.708.708l-2-2zM10 8a.5.5 0 01.5-.5h3.793l-1.147-1.146a.5.5 0 01.708-.708l2 2a.5.5 0 010 .708l-2 2a.5.5 0 01-.708-.708L14.293 8.5H10.5A.5.5 0 0110 8z"
                />
            </svg>
        </div>
        <Nipple options={{ size: props.size }} style={{ width: "100%", height: "100%" }}
                onStart={handleJoystickStart}
                onEnd={handleJoystickEnd}
                onMove={handleJoystickMove} />
    </Card>;
}