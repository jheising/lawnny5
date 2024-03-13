import { useEffect, useRef, useState } from "react";
import equal from "fast-deep-equal";
import { useStateCallback } from "./useStateCallback";
import { JoystickPosition } from "../components/JoystickPad";

export interface GamepadInfo {
    readonly axes: ReadonlyArray<number>;
    readonly buttons: ReadonlyArray<boolean>;
}

export interface UseGamepadDispatch {
    connected: boolean;
    gamepad?: Gamepad;
}

export interface UseGamepadProps {
    enableLatching?: boolean;
    onInputChanged?: (info?: GamepadInfo) => void;
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
    const gamepadInfo = useStateCallback<GamepadInfo>({
        onValueChanged: (value) => {
            if (propsRef.current?.onJoystickPosition) {
                propsRef.current.onJoystickPosition({
                    xPercent: value?.axes[2] ?? 0,
                    yPercent: -(value?.axes[3] ?? 0)
                });
            }
        },
        shouldRepeat: (value) => {
            if (!value || gamepadIndex.current === undefined) {
                return false;
            }

            // Are any buttons pressed?
            // if (value.buttons.some(button => button)) {
            //     return true;
            // }

            // Are any axes greater than our deadzone?
            const deadZone = propsRef.current?.deadZone ?? 0.009;
            if (value.axes.some(axes => Math.abs(axes) > deadZone)) {
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

        gamepadInfo.updateValue({
            axes: gamepad.axes,
            buttons: convertButtonsToArray(gamepad.buttons)
        });

        setTimeout(pollingLoop);
    }

    return {
        connected: isConnected,
        gamepad: gamepad
    };
}