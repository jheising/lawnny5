import { StyleSheet } from "react-native";

export const COLOR_BG = "#1B3B5C";
export const COLOR_PRIMARY = "#4EF5BF";
export const COLOR_LIGHT = "#FFFFFF";

const styles = StyleSheet.create({
    background: {
        backgroundColor: COLOR_BG
    },
    textLight: {
        color: COLOR_LIGHT
    },
    fullHeight: {
        height: "100%"
    },
    textPrimary: {
        color: COLOR_PRIMARY
    }
});

export default styles;