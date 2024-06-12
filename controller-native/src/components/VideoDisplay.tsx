import { Message, Ros, Topic } from "roslib";
import { useEffect, useRef, useState } from "react";
import { Image, StyleProp, View } from "react-native";
import { useInterval } from "odyssey-ui/dist/hooks/useInterval";
import { BlinkAnimation, Box, Text, useTheme } from "odyssey-ui";
import { ViewStyle } from "react-native/Libraries/StyleSheet/StyleSheetTypes";

export interface VideoDisplayProps {
    ros: Ros;
    style?: StyleProp<ViewStyle>;
}

interface HandTrackingMessage {
    bound_height: number;
    bound_width: number;
    height: number;
    name: number;
    width: number;
    x: number;
    y: number;
}

export function VideoDisplay(props: VideoDisplayProps) {
    const theme = useTheme();
    const imageRef = useRef<Image>(null);
    const lastImageTime = useRef<number>();
    const lastHandTime = useRef<number>();
    const [imageData, setImageData] = useState<{ uri: string }>();
    const [handMessage, setHandMessage] = useState<HandTrackingMessage>();
    useInterval(() => {
        const now = Date.now();
        if (lastImageTime.current && now - lastImageTime.current >= 500) {
            setImageData(undefined);
            lastImageTime.current = undefined;
        }

        if (lastHandTime.current && now - lastHandTime.current >= 500) {
            setHandMessage(undefined);
            lastHandTime.current = undefined;
        }
    }, 1000, true);

    useEffect(() => {
        if (props.ros) {
            function handleVideoMessage(message: any) {
                setImageData({ uri: `data:image/jpeg;base64,` + message.data });
                lastImageTime.current = Date.now();
            }

            function handleHandTrackingMessage(message: Message) {
                setHandMessage(message as HandTrackingMessage);
                lastHandTime.current = Date.now();
            }

            const videoListener = new Topic({
                ros: props.ros,
                name: "camera/color/bgr",
                messageType: "sensor_msgs/CompressedImage"
            });
            videoListener.subscribe(handleVideoMessage);

            const handTrackListener = new Topic({
                ros: props.ros,
                name: "hand_tracking",
                messageType: "lawnny5_interfaces/msg/ObjectTrack"
            });
            handTrackListener.subscribe(handleHandTrackingMessage);

            return () => {
                videoListener.unsubscribe(handleVideoMessage);
                handTrackListener.unsubscribe(handleHandTrackingMessage);
            };
        }
    }, [props.ros]);

    return imageData?.uri ? <View style={[props.style as any]}>
            <Image ref={imageRef} style={{ flex: 1 }} source={imageData} />
            {!!handMessage && <BlinkAnimation animationDelay={0}>
                <View style={{ position: "absolute", bottom: 20, right: 20, backgroundColor: theme.settings.backgroundColor, padding: 5 }}>
                    <Text>TRACKING HAND</Text>
                </View>
            </BlinkAnimation>}
            {/*{!!handMessage && <View*/}
            {/*    style={{ position: "absolute", bottom: 20, right: 20, width: 10, height: 10, backgroundColor: "red", left: `${handMessage.x / handMessage.bound_width * 100}%`, top: `${handMessage.y / handMessage.bound_height * 100}%` }} />}*/}
        </View>
        :
        <Box style={{ flex: 1, justifyContent: "center", alignItems: "center" }}>
            <BlinkAnimation><Text style={{ fontSize: 20 }}>NO SIGNAL</Text></BlinkAnimation>
        </Box>;
}