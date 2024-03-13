import { useEffect, useRef } from "react";
import equal from "fast-deep-equal";

export interface UseStateCallbackProps<T> {
    onValueChanged?: (newValue?: T) => void;
    shouldRepeat?: (value?: T) => boolean;
    repeatIntervalInMS?: number;
}

export interface UseStateCallbackDispatch<T> {
    updateValue: (newValue: T) => void;
}

export function useStateCallback<T>(props: UseStateCallbackProps<T>): UseStateCallbackDispatch<T> {
    const lastProps = useRef(props);
    const lastValue = useRef<T>();
    const currentRepeatLoop = useRef<number>();

    function startRepeatLoop() {
        if (currentRepeatLoop.current) {
            return;
        }

        processRepeatLoop();
    }

    function stopRepeatLoop() {
        if (currentRepeatLoop.current) {
            clearTimeout(currentRepeatLoop.current);
            currentRepeatLoop.current = undefined;
        }
    }

    function processUpdateValue() {
        if (lastProps.current.onValueChanged) {
            lastProps.current.onValueChanged(lastValue.current);
        }
    }

    function processRepeatLoop() {
        if (!lastProps.current.repeatIntervalInMS || !lastProps.current.shouldRepeat) {
            return;
        }

        // Should we repeat updating this value?
        if (!lastProps.current.shouldRepeat(lastValue.current)) {
            currentRepeatLoop.current = undefined;
            return;
        }

        processUpdateValue();
        currentRepeatLoop.current = window.setTimeout(processRepeatLoop, lastProps.current.repeatIntervalInMS);
    }

    function updateValue(newValue: T) {

        const valueChanged = !equal(newValue, lastValue.current);
        lastValue.current = newValue;

        // If the value has changed, notify the callback
        if (valueChanged) {
            processUpdateValue();
            startRepeatLoop();
        }
    }

    useEffect(() => {
        return stopRepeatLoop;
    }, []);

    useEffect(() => {
        lastProps.current = props;
    }, [props]);

    return {
        updateValue
    };
}