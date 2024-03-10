import { Ros, Topic } from "roslib";
import { useEffect, useRef } from "react";

export interface VideoDisplayProps {
    ros: Ros;
    className?: string;
}

export function VideoDisplay(props: VideoDisplayProps) {
    const imageRef = useRef<HTMLImageElement>(null);

    useEffect(() => {
        if (props.ros) {
            function handleVideoMessage(message: any) {
                if (imageRef.current) {
                    imageRef.current.src = `data:image/png;base64, ` + message.data;
                }
            }

            const videoListener = new Topic({
                ros: props.ros,
                name: "camera/color/bgr",
                messageType: "sensor_msgs/CompressedImage"
            });

            videoListener.subscribe(handleVideoMessage);

            return () => {
                videoListener.unsubscribe(handleVideoMessage);
            };
        }
    }, [props.ros]);

    return <img className={props.className} ref={imageRef} alt="" />;
}