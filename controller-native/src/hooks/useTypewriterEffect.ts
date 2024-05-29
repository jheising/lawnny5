import { useEffect, useRef, useState } from "react";

export interface UseTypewriterEffectProps {
    rateInMS?: number;
    enabled?: boolean;
}

export interface UseTypewriterEffectDispatch {
    text?: string;
    setText: (value: string) => void;
    isFinished: boolean;
}

export function useTypewriterEffect(props?: UseTypewriterEffectProps): UseTypewriterEffectDispatch {

    const [currentText, setCurrentText] = useState<string>();
    const [isFinished, setIsFinished] = useState(true);
    const [targetText, setTargetText] = useState<string>();
    const currentTextRef = useRef<string>();
    const timer = useRef<any>();
    const enabled = props?.enabled !== false;

    useEffect(() => {
        reset();

        if (props?.enabled !== false) {
            begin();
        }

    }, [enabled, targetText]);

    function reset() {
        currentTextRef.current = "";
        setCurrentText("");
        setIsFinished(false);

        if (timer.current) {
            clearInterval(timer.current);
            timer.current = undefined;
        }
    }

    function begin() {
        if (!targetText) {
            return;
        }

        timer.current = setInterval(() => {
            const newLength = currentTextRef.current!.length + 1;
            const newText = targetText.substring(0, newLength);
            currentTextRef.current = newText;
            setCurrentText(newText);

            if (newText === targetText) {
                setIsFinished(true);
                clearInterval(timer.current);
                timer.current = undefined;
            }

        }, props?.rateInMS ?? 50);
    }

    function setText(value: string) {
        setTargetText(value);
    }

    return {
        text: currentText,
        setText,
        isFinished
    };
}