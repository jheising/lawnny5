import { Box, Group, Subtitle, Text, Title } from "odyssey-ui";
import { UseROSDispatch } from "../../hooks/useROS";
import { Video, ResizeMode } from "expo-av";
import { View, StyleSheet, Image } from "react-native";
import { useRef } from "react";
import { CRTScreen } from "../CRTScreen";
import { ChatBox } from "../ChatBox";
import { LoadingScreen } from "../LoadingScreen";

export interface KioskControllerProps {
    ros: UseROSDispatch;
}

export function KioskController(props: KioskControllerProps) {
    const video = useRef(null);

    function renderChatWindow() {
        if (!props.ros || !props.ros.isConnected) {
            return <View style={{width: "33%", height: "100%", justifyContent: "center", alignItems: "center"}}>
                <LoadingScreen />
            </View>
        }

        return <Group style={{
            width: "33%",
            padding: 25,
            height: "100%",
            flexDirection: "column"
        }}>
            <Text style={{ fontSize: 25 }}>CHAT WITH LAWNNY 5 JR.</Text>
            {props.ros.ros && <ChatBox ros={props.ros.ros} style={{ flex: 1 }} />}
        </Group>;
    }

    return <CRTScreen>
        <View style={{
            width: "100%",
            height: "100%",
            flexDirection: "row"
        }}>
            <Video
                ref={video}
                style={{
                    flex: 1
                }}
                videoStyle={{
                    width: "100%",
                    height: "100%"
                }}
                source={require("../../../assets/l5-demo.mp4")}
                isMuted
                shouldPlay
                resizeMode={ResizeMode.CONTAIN}
                isLooping
            />
            {renderChatWindow()}
        </View>
        <View style={{
            backgroundColor: "white",
            overflow: "hidden",
            borderRadius: 25,
            position: "absolute",
            bottom: 25,
            left: 25
        }}>
            <Image source={require("../../../assets/qrcode.png")} resizeMode="contain" style={{ width: 320, height: 320 }} />
        </View>
    </CRTScreen>;
}