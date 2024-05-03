import { StyleSheet, View } from "react-native";
import { Page, Themes, Field, Switch, ProgressCircle, Group, Dropdown, Button, Option } from "odyssey-ui";
import { Joypad, JoystickPosition } from "./Joypad";
import { useEffect, useRef, useState } from "react";
import { UseROSDispatch } from "../hooks/useROS";
import { Message, Topic } from "roslib";

export interface ControllerProps {
    ros: UseROSDispatch;
}

type NavMode = "JOYSTICK" | "FOLLOW_ME";
type NavModeOption = { name: string, value: NavMode };
const NAV_MODE_OPTIONS: NavModeOption[] = [
    { name: "MANUAL", value: "JOYSTICK" },
    { name: "FOLLOW ME", value: "FOLLOW_ME" }
];

export function Controller(props: ControllerProps) {
    const [joystickPosition, setJoystickPosition] = useState<JoystickPosition>();

    const xPercent = Math.round((joystickPosition?.xPercent ?? 0) * 100);
    const yPercent = Math.round((joystickPosition?.yPercent ?? 0) * 100);

    const [navMode, setNavMode] = useState<NavModeOption | undefined>();
    const joystickPublisher = useRef<Topic>();

    useEffect(() => {
        if (props.ros.isConnected) {
            loadSettings();

            joystickPublisher.current = new Topic({
                ros: props.ros.ros!,
                name: "joy",
                messageType: "sensor_msgs/Joy"
            });

            return props.ros.onSettingChange(handleSettingChange);
        }
    }, [props.ros.isConnected]);

    useEffect(() => {

    }, [navMode]);

    async function loadSettings() {
        handleSettingChange("nav-mode", await props.ros.getSetting<NavMode>("nav-mode"));
    }

    function handleSettingChange(name: string, value: any) {
        switch (name) {
            case "nav-mode": {
                setNavMode(NAV_MODE_OPTIONS.find(nm => nm.value === value));
                break;
            }
        }
    }

    function handleJoystickMove(position?: JoystickPosition) {
        setJoystickPosition(position);
        if (joystickPublisher.current && position) {
            joystickPublisher.current.publish(new Message({
                axes: [position.xPercent, position.xPercent],
                buttons: []
            }));
        }
    }

    function handleNavModeChange(option?: Option) {
        if (option) {
            props.ros.setSetting("nav-mode", option.value, true);
        }
    }

    return <View style={{ minHeight: "100%", width: "100%", maxWidth: 1024, padding: 10, paddingBottom: 0 }}>
        <Page title="LNE5" description="VITA AG SYSTEMS L-SERIES MK5">
            <View style={{ width: "100%", height: "100%", display: "flex", flexDirection: "column" }}>
                <View style={{ flex: 1 }}>
                </View>
                <Group direction="vertical" title="NAV MODE">
                    <Dropdown placeholderText="NAV MD" value={navMode} options={NAV_MODE_OPTIONS} onChange={handleNavModeChange} />
                    <Field label="JOYPAD">
                        <View style={{ height: 300 }}>
                            <Joypad style={{ position: "absolute", top: 0, left: 0, right: 0, bottom: 0 }} onMove={handleJoystickMove} />
                            {/*<Animated.View style={[{*/}
                            {/*    position: "absolute",*/}
                            {/*    backgroundColor: Themes.Slate.primaryColor,*/}
                            {/*    height: MyTheme.scale * 2*/}
                            {/*}, joystickXAnimation]} />*/}
                            {/*<Animated.View style={[{*/}
                            {/*    position: "absolute",*/}
                            {/*    backgroundColor: Themes.Slate.primaryColor,*/}
                            {/*    width: MyTheme.scale * 2*/}
                            {/*}, joystickYAnimation]} />*/}
                            <View style={{ position: "absolute", bottom: 0, right: 0, pointerEvents: "none" }}>
                                <Group direction="horizontal">
                                    <ProgressCircle title="ANG-V" style="center" colorVariant={xPercent >= 0 ? "primary" : "warn"} fill={Math.abs(xPercent)} displayValue={xPercent + "%"} animate={false} />
                                    <ProgressCircle title="LIN-V" style="center" colorVariant={yPercent >= 0 ? "primary" : "warn"} fill={Math.abs(yPercent)} displayValue={yPercent + "%"} animate={false} />
                                </Group>
                            </View>
                        </View>
                    </Field>
                    <Button title="ESTOP" colorVariant="warn" />
                </Group>
            </View>
        </Page>
    </View>;
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        overflow: "hidden",
        backgroundColor: Themes.Slate.backgroundColor
    }
});
