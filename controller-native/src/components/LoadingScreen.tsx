import React, { useEffect, useRef, useState } from "react";
import { Image, View } from "react-native";
import { BlinkAnimation, Button, Group, Subtitle, Switch, Tag, Title, Value } from "odyssey-ui";
import Animated, { useSharedValue, withRepeat, withTiming, Easing, useAnimatedStyle } from "react-native-reanimated";

const images = [require("../../assets/anim1.webp"), require("../../assets/anim2.webp")];

function randomInt(min: number, max: number) { // min and max included
    return Math.floor(Math.random() * (max - min + 1) + min);
}

export function LoadingScreen() {

    const [randomValue, setRandomValue] = useState(0);
    const [image, setImage] = useState();
    const timer = useRef<any>();
    const blink = useSharedValue(0.0);
    const blinkStyle = useAnimatedStyle(() => ({ opacity: blink.value }), []);

    useEffect(() => {
        handleRandomChange();

        blink.value = withRepeat(
            withTiming(1, { duration: 500, easing: Easing.back() }),
            -1,
            true
        );

        return () => {
            if (timer.current) {
                clearTimeout(timer.current);
                timer.current = undefined;
            }
        };
    }, []);

    function handleRandomChange() {
        const newImage = images[randomInt(0, images.length - 1)];
        setRandomValue(-randomInt(0, 100) / 10);
        setImage(newImage);

        timer.current = setTimeout(handleRandomChange, randomInt(5000, 30000));
    }

    return <View style={{ display: "flex", flexDirection: "row", alignItems: "center", justifyContent: "center", gap: 40, flexWrap: "wrap" }}>
        <BlinkAnimation key={randomValue}>
            <Image style={{
                width: 320,
                height: 320,
                resizeMode: "contain"
            }} source={image} />
        </BlinkAnimation>
        <Group>
            <Title label="DNLK" />
            <Value label="STAT" value="DWN" colorVariant="danger" />
            <Value label="ANRAD" value={{
                number: randomValue,
                suffix: " dBi"
            }} />
            <Animated.View style={blinkStyle}>
                <Tag label="SRCH..." colorVariant="warn" />
            </Animated.View>
        </Group>
    </View>;
}