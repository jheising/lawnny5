import { StyleSheet, useWindowDimensions } from "react-native";
import { Theme, Themes } from "odyssey-ui";
import { useEffect, useState } from "react";
import { useROS } from "./src/hooks/useROS";
import { LoadingScreen } from "./src/components/LoadingScreen";
import { CRTScreen } from "./src/components/CRTScreen";
import { Controller } from "./src/components/Controller";

const MyTheme = Themes.Slate;

const DEFAULT_ROS_PORT = 9090;

export default function App() {

    const [rosURL, setROSURL] = useState<string>();
    const { height, width } = useWindowDimensions();
    const ros = useROS({
        rosURL: rosURL,
        enabled: true
    });

    useEffect(() => {
        // Automatically connect to the ROS server running on this device when the page loads
        setROSURL(`ws://${document.location.hostname}:${DEFAULT_ROS_PORT}`);
        //setROSURL(`ws://debian.local:${DEFAULT_ROS_PORT}`);
    }, []);

    return (<Theme theme={{
        ...MyTheme,
        scale: height >= 1024 ? 2.5 : 2.0
    }}>
        <CRTScreen>
            {ros.isConnected ? <Controller ros={ros} /> : <LoadingScreen />}
        </CRTScreen>
    </Theme>);
}

const styles = StyleSheet.create({
    container: {
        flex: 1,
        overflow: "hidden",
        backgroundColor: Themes.Slate.backgroundColor
    }
});
