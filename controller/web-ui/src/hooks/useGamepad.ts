import { useEffect, useRef, useState } from "react";
import { useStateCallback } from "./useStateCallback";
import { JoystickPosition } from "../components/JoystickPad";

export interface UseGamepadDispatch {
    connected: boolean;
    gamepad?: Gamepad;
}

export interface UseGamepadProps {
    enableLatching?: boolean;
    onJoystickPosition?: (position?: JoystickPosition) => void;
    deadZone?: number;
}

function getButtonValue(button: GamepadButton): boolean {
    return button.pressed || button.touched;
}

function convertButtonsToArray(buttons: ReadonlyArray<GamepadButton>): boolean[] {
    return buttons.map(getButtonValue);
}

export function useGamepad(props?: UseGamepadProps): UseGamepadDispatch {
    const [isConnected, setIsConnected] = useState(false);
    const [gamepad, setGamepad] = useState<Gamepad>();
    const gamepadIndex = useRef<number>();
    const propsRef = useRef(props);
    const joystickState = useStateCallback<ReadonlyArray<number>>({
        onValueChanged: (value) => {
            if (propsRef.current?.onJoystickPosition) {
                propsRef.current.onJoystickPosition({
                    xPercent: value?.[2] ?? 0,
                    yPercent: -(value?.[3] ?? 0)
                });
            }
        },
        shouldRepeat: (value) => {
            if (!value || gamepadIndex.current === undefined) {
                return false;
            }

            // Are any axes greater than our deadzone?
            const deadZone = propsRef.current?.deadZone ?? 0.009;
            if (value.some(axis => Math.abs(axis) > deadZone)) {
                return true;
            }

            return false;
        },
        repeatIntervalInMS: 150
    });

    useEffect(() => {
        propsRef.current = props;
    }, [props]);

    useEffect(() => {

        function gamepadConnected(event: GamepadEvent) {
            setIsConnected(true);
            setGamepad(event.gamepad);
            gamepadIndex.current = event.gamepad.index;
            pollingLoop();
        }

        function gamepadDisconnected(event: GamepadEvent) {
            setIsConnected(false);
            setGamepad(undefined);
            gamepadIndex.current = undefined;
        }

        window.addEventListener("gamepadconnected", gamepadConnected);
        window.addEventListener("gamepaddisconnected", gamepadDisconnected);

        return () => {
            window.removeEventListener("gamepadconnected", gamepadConnected);
            window.removeEventListener("gamepaddisconnected", gamepadDisconnected);
        };
    }, []);

    function pollingLoop() {
        if (gamepadIndex.current === undefined) {
            return;
        }

        const gamepads = navigator.getGamepads();

        if (!gamepads) {
            return;
        }

        const gamepad = gamepads[gamepadIndex.current];

        if (!gamepad) {
            return;
        }

        joystickState.updateValue(gamepad.axes);

        setTimeout(pollingLoop);
    }

    return {
        connected: isConnected,
        gamepad: gamepad
    };
}