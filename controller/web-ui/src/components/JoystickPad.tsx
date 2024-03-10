import Nipple, { EventData, JoystickOutputData } from "react-nipplejs";
import { useEffect, useRef } from "react";

export interface JoystickPosition {
    xPercent: number;
    yPercent: number;
}

export interface JoystickPadProps {
    size: number;
    onJoystickPosition?: (position?: JoystickPosition) => void;
    outputInterval?: number;
}

export function JoystickPad(props: JoystickPadProps) {
    const joystickContainerRef = useRef<HTMLDivElement>(null);
    const isJoystickActive = useRef(false);
    const joystickPosition = useRef<JoystickPosition>({ xPercent: 0, yPercent: 0 });

    useEffect(() => {
        return endJoystickLatch;
    }, []);

    function startJoystickLatch() {
        isJoystickActive.current = true;

        const processJoystickLatch = () => {
            if (isJoystickActive.current && joystickContainerRef.current) {
                sendCurrentJoystickPosition();
                setTimeout(processJoystickLatch, props.outputInterval ?? 100);
            }
        };
        processJoystickLatch();
    }

    function endJoystickLatch() {
        isJoystickActive.current = false;
    }

    function sendCurrentJoystickPosition() {
        if (props.onJoystickPosition) {
            props.onJoystickPosition(joystickPosition.current);
        }
    }

    function saveJoystickData(data: JoystickOutputData) {
        const posX = data.distance * Math.cos(data.angle.radian);
        const posY = data.distance * Math.sin(data.angle.radian);
        const halfJoystickSize = props.size / 2;

        const xPercent = posX / halfJoystickSize;
        const yPercent = posY / halfJoystickSize;

        joystickPosition.current = {
            xPercent: xPercent,
            yPercent: yPercent
        };
    }

    function handleJoystickStart(event: EventData, data: JoystickOutputData) {
        startJoystickLatch();
    }

    function handleJoystickEnd(event: EventData, data: JoystickOutputData) {
        endJoystickLatch();
    }

    function handleJoystickMove(event: EventData, data: JoystickOutputData) {
        saveJoystickData(data);
        sendCurrentJoystickPosition();
    }

    return <div className="w-full h-full" ref={joystickContainerRef}>
        <Nipple options={{ size: props.size }} style={{ width: "100%", height: "100%" }}
                onStart={handleJoystickStart}
                onEnd={handleJoystickEnd}
                onMove={handleJoystickMove} />
    </div>;
}