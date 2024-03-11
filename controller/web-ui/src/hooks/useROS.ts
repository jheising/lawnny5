import { useEffect, useRef, useState } from "react";
import ROSLIB, { Ros } from "roslib";

export interface UseROSProps {
    rosURL?: string;
    enabled?: boolean;
}

export interface UseROSDispatch {
    isConnected: boolean;
    ros?: Ros;
    lastError?: any;
}

export function useROS(props: UseROSProps): UseROSDispatch {
    const ros = useRef<Ros>();
    const [isConnected, setIsConnected] = useState(false);
    const [lastError, setLastError] = useState<any>();

    useEffect(() => {

        if (ros.current) {
            ros.current?.close();
            ros.current = undefined;
        }

        setLastError(undefined);

        if (props.enabled && props.rosURL) {
            ros.current = new ROSLIB.Ros({
                url: props.rosURL
            });

            ros.current.on("connection", function() {
                setIsConnected(true);
            });

            ros.current.on("error", function(error) {
                setLastError(error);
            });

            ros.current.on("close", function() {
                setIsConnected(false);
            });
        }
    }, [props.enabled, props.rosURL]);

    return {
        isConnected,
        ros: ros.current,
        lastError
    };
}