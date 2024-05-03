import { StyleSheet, View } from "react-native";
import { GestureDetector, Gesture } from "react-native-gesture-handler";
import Animated, {
    useSharedValue,
    useAnimatedStyle, StyleProps
} from "react-native-reanimated";
import { useRef, useState } from "react";
import { useStateCallback } from "../hooks/useStateCallback";

export interface JoystickPosition {
    xPercent: number;
    yPercent: number;
}

export interface JoypadProps {
    dimpleSize?: number;
    padSize?: number;
    onMove?: (position?: JoystickPosition) => void;
    onEnd?: () => void;
    style?: StyleProps;

    outputInterval?: number;
    //deadZone?: number;
}

function clamp(value: number, min: number, max: number) {
    return Math.min(max, Math.max(min, value));
}

export function Joypad(props: JoypadProps) {

    const padSize = props.padSize ?? 300;
    const halfSize = padSize / 2;
    const dimpleSize = props.dimpleSize ?? 150;
    const dimpleHalfSize = dimpleSize / 2.0;
    const panGesture = Gesture.Pan()
        .minDistance(0)
        .onBegin((e) => {
            dragStartPosition.current = {
                x: e.x - dimpleHalfSize,
                y: e.y - dimpleHalfSize
            };
            dragCurrentPosition.value = {
                x: 0,
                y: 0
            };
            setIsDragging(true);
        })
        .onUpdate((e) => {

            joystickState.updateValue({
                xPercent: clamp(e.translationX / halfSize, -1.0, 1.0),
                yPercent: -clamp(e.translationY / halfSize, -1.0, 1.0)
            });

            dragCurrentPosition.value = {
                x: e.translationX + dragStartPosition.current.x,
                y: e.translationY + dragStartPosition.current.y
            };
        })
        .onEnd((e) => {
            setIsDragging(false);

            if (props.onMove) {
                props.onMove({ xPercent: 0.0, yPercent: 0.0 });
            }

            if (props.onEnd) {
                props.onEnd();
            }
        });

    const [isDragging, setIsDragging] = useState(false);
    const dragStartPosition = useRef<{ x: number, y: number }>({ x: 0, y: 0 });
    const dragCurrentPosition = useSharedValue<{ x: number, y: number }>({ x: 0, y: 0 });
    const animatedStyle = useAnimatedStyle(() => ({
        transform: [
            { translateX: dragCurrentPosition.value.x },
            { translateY: dragCurrentPosition.value.y }
        ]
    }));

    const joystickState = useStateCallback({
        onValueChanged: props.onMove,
        repeatIntervalInMS: props.outputInterval ?? 150,
        shouldRepeat: (value) => {
            if (!isDragging || !value) {
                return false;
            }

            //const deadZone = props.deadZone ?? 0.009;

            // if (Math.abs(value.xPercent) > deadZone || Math.abs(value.yPercent) > deadZone) {
            //     return true;
            // }

            return true;
        }
    });

    return <GestureDetector gesture={panGesture}>
        <View style={[{ position: "relative", cursor: "move" }, props.style]}>
            {isDragging && <Animated.View style={[styles.dragIndicator, { width: dimpleSize, height: dimpleSize }, animatedStyle]} />}
        </View>
    </GestureDetector>;
}

const styles = StyleSheet.create({
    dragIndicator: {
        position: "absolute",
        backgroundColor: "white",
        borderRadius: 1000,
        zIndex: 100,
        opacity: 0.1
    }
});