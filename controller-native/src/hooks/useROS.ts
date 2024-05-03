import { useEffect, useRef, useState } from "react";
import ROSLIB, { Message, Ros, Service, ServiceRequest, Topic } from "roslib";

export interface UseROSProps {
    rosURL?: string;
    enabled?: boolean;
}

export interface UseROSDispatch {
    isConnected: boolean;
    ros?: Ros;
    lastError?: any;
    getSettings: <T extends Record<string, any> = Record<string, any>>() => Promise<T>;
    setSetting: (name: string, value: any, temporary?: boolean) => Promise<void>;
    getSetting: <T = any>(name: string, defaultValue?: T) => Promise<T | undefined>;
    onSettingChange: (callback: (name: string, value: any) => void) => () => void;
}

async function callService(service: Service, params: Record<string, any>): Promise<any> {
    return new Promise((resolve, reject) => {
        service.callService(new ServiceRequest(params), resolve, reject);
    });
}

export function useROS(props: UseROSProps): UseROSDispatch {
    const ros = useRef<Ros>();
    const services = useRef<Record<"get_setting" | "set_setting" | "get_settings", Service>>({} as any);
    const settingsChangeSubscription = useRef<Topic>();
    const [isConnected, setIsConnected] = useState(false);
    const [lastError, setLastError] = useState<any>();

    useEffect(() => {
        startConnection();
    }, [props.enabled, props.rosURL]);

    function endConnection() {
        if (ros.current) {
            ros.current?.close();
            ros.current = undefined;
        }
        setIsConnected(false);
        setLastError(undefined);
    }

    function tryConnection() {
        if (ros.current && props.enabled && props.rosURL) {
            console.log("Connecting to ROS");
            ros.current?.connect(props.rosURL);
        }
    }

    function startConnection() {
        endConnection();

        if (props.enabled && props.rosURL) {
            ros.current = new ROSLIB.Ros({});

            services.current.get_setting = new Service({
                ros: ros.current,
                name: "get_setting",
                serviceType: "lawnny5_interfaces/srv/GetGlobalSetting"
            });

            services.current.set_setting = new Service({
                ros: ros.current,
                name: "set_setting",
                serviceType: "lawnny5_interfaces/srv/SetGlobalSetting"
            });

            services.current.get_settings = new Service({
                ros: ros.current,
                name: "get_settings",
                serviceType: "lawnny5_interfaces/srv/GetGlobalSettings"
            });

            settingsChangeSubscription.current = new ROSLIB.Topic({
                ros: ros.current,
                name: "global_setting_updated",
                messageType: "diagnostic_msgs/KeyValue"
            });

            ros.current.on("connection", function() {
                setIsConnected(true);
            });

            ros.current.on("error", function(error) {
                setLastError(error);
            });

            ros.current.on("close", function() {
                setIsConnected(false);
                setTimeout(tryConnection, 3000);
            });

            tryConnection();
        }
    }


    async function getSettings<T extends Record<string, any> = Record<string, any>>(): Promise<T> {
        if (services.current.get_settings) {
            const results = await callService(services.current.get_settings, {});
            return JSON.parse(results.value);
        }

        throw new Error("Not connected");
    }

    async function getSetting<T = any>(name: string, defaultValue?: T): Promise<T | undefined> {
        if (services.current.get_setting) {
            const results = await callService(services.current.get_setting, { name });

            if (results.value === "undefined") {
                return defaultValue;
            }

            try {
                return JSON.parse(results.value);
            } catch (e) {
                return results.value;
            }
        }

        throw new Error("Not connected");
    }

    async function setSetting(name: string, value: any, temporary: boolean = false) {
        if (services.current.set_setting) {
            await callService(services.current.set_setting, { name, value, temporary });
            return;
        }

        throw new Error("Not connected");
    }

    function onSettingChange(callback: (name: string, value: any) => void): () => void {
        if (settingsChangeSubscription.current) {
            const callbackWrapper = (message: Message) => {
                callback((message as any).key, (message as any).value);
            };
            settingsChangeSubscription.current.subscribe(callbackWrapper);
            return () => {
                if (settingsChangeSubscription.current) {
                    settingsChangeSubscription.current.unsubscribe(callbackWrapper);
                }
            };
        }

        throw new Error("Not connected");
    }

    return {
        isConnected,
        ros: ros.current,
        lastError,
        getSettings,
        getSetting,
        setSetting,
        onSettingChange
    };
}