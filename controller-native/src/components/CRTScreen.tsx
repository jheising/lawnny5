import { View } from "react-native";
import { ScreenOverlay, useTheme } from "odyssey-ui";
import { PropsWithChildren, useEffect, useState } from "react";
import Animated, { Easing, useAnimatedStyle, useSharedValue, withDelay, withTiming } from "react-native-reanimated";

export function CRTScreen(props: PropsWithChildren) {
    const theme = useTheme();
    const [bootAnimationFinished, setBootAnimationFinished] = useState(false);
    const widthValue = useSharedValue(0);
    const heightValue = useSharedValue(5);
    const animationStyle = useAnimatedStyle(() => {
        return {
            height: heightValue.value > 5 ? `${heightValue.value}%` : 10,
            width: `${widthValue.value}%`
        };
    });

    useEffect(() => {
        widthValue.value = withTiming(100, { duration: 300 });
        heightValue.value = withDelay(300, withTiming(100, { duration: 300 }, () => {
            setBootAnimationFinished(true);
        }));
    }, []);

    return <ScreenOverlay style={{overflow:"hidden"}} contentContainerStyle={{ flex: 1, backgroundColor: bootAnimationFinished ? theme.backgroundColor : "#000000", display: "flex", justifyContent: "center", alignItems: "center" }}>
        {bootAnimationFinished ? props.children : <Animated.View style={[{ backgroundColor: theme.backgroundColor }, animationStyle]} />}
    </ScreenOverlay>;
}