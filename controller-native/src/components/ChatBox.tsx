import { Message, Ros, Topic } from "roslib";
import { StyleProp, View, TextInput, NativeSyntheticEvent, TextInputKeyPressEventData } from "react-native";
import { Text, Chip, Group, Subtitle, useTheme } from "odyssey-ui";
import { useEffect, useRef, useState } from "react";
import { useTypewriterEffect } from "../hooks/useTypewriterEffect";
import { ViewStyle } from "react-native/Libraries/StyleSheet/StyleSheetTypes";

export interface ChatBoxProps {
    ros: Ros;
    style?: StyleProp<ViewStyle>;
}

export function ChatBox(props: ChatBoxProps) {
    const theme = useTheme();
    const inputRef = useRef<TextInput>(null);
    const [chatInputText, setChatInputText] = useState("");
    const chatPublisher = useRef<Topic>();
    const chatRequestText = useTypewriterEffect({});
    const chatResponseText = useTypewriterEffect({
        enabled: chatRequestText.isFinished
    });

    useEffect(() => {
        if (props.ros) {
            function handleChatResponse(message: any) {
                chatResponseText.setText(message.data);
            }

            const chatResponseListener = new Topic({
                ros: props.ros,
                name: "personality/chat/output/full",
                messageType: "std_msgs/String"
            });
            chatResponseListener.subscribe(handleChatResponse);

            chatPublisher.current = new Topic({
                ros: props.ros,
                name: "personality/chat/input",
                messageType: "lawnny5_interfaces/msg/Chat"
            });

            return () => {
                chatResponseListener.unsubscribe(handleChatResponse);
            };
        }
    }, [props.ros]);

    function handleSubmit() {
        chatRequestText.setText(chatInputText);
        chatResponseText.setText("");
        setChatInputText("");

        if (chatPublisher.current) {
            chatPublisher.current.publish(new Message({
                data: chatInputText,
                generate_tts: true,
                allow_movement: true
            }));
        }
    }

    return <View style={[props.style]}>
        <Chip outlined={true} style={{ minHeight: theme.controlMinHeight, justifyContent: "center", padding: theme.innerPadding, paddingRight: 0 }}>
            <View style={{
                flex: 1,
                borderBottomColor: theme.color,
                borderBottomWidth: theme.borderWidth
            }}>
                <TextInput ref={inputRef}
                           autoFocus
                           placeholder="INPUT..."
                           placeholderTextColor={"#FFFFFF50"}
                            // @ts-ignore
                           style={{ flex: 1, fontFamily: "odyssey", color: theme.color, fontSize: theme.fontSize, outline: "none", textTransform: "uppercase" }}
                           value={chatInputText}
                           onChangeText={setChatInputText}
                           blurOnSubmit={false}
                           onSubmitEditing={handleSubmit}
                           returnKeyType={"send"} />
            </View>
        </Chip>
        <View style={{ flex: 1, justifyContent: "center", alignItems: "center" }}>
            <Text style={{ textTransform: "uppercase", textAlign: "center", fontSize: theme.fontSize + 2, marginBottom: 50 }}>{chatRequestText.text}</Text>
            <Text style={{ textTransform: "uppercase", fontSize: theme.fontSize + 5, textAlign: "center"}}>{chatResponseText.text}</Text>
        </View>
    </View>;
}