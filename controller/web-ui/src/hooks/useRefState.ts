import { Dispatch, SetStateAction, useRef, useState } from "react";

export function useRefState<S>(initialState: S | (() => S)): [S, (newValue: S) => void, S] {
    const [value, _setValue] = useState<S>(initialState);

    const valueRef = useRef<S>(value);
    const setValue = (value: S) => {
        valueRef.current = value;
        _setValue(value);
    };

    return [value, setValue, valueRef.current];
}