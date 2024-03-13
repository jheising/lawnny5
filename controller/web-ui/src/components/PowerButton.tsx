import { IconButton } from "ui-neumorphism";

export interface PowerButtonProps {
    on: boolean;
    onClick?: () => void;
}

export function PowerButton(props: PowerButtonProps) {
    return <IconButton dark rounded bordered text={false} size="large" onClick={props.onClick} className="relative">
        <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" strokeWidth={1.5} stroke="currentColor" className="w-6 h-6">
            <path strokeLinecap="round" strokeLinejoin="round" d="M5.636 5.636a9 9 0 1 0 12.728 0M12 3v9" />
        </svg>
        {props.on && <div className="absolute w-full h-full rounded-full border-2 border-sky-200 shadow-[0_0_2px_#fff,inset_0_0_2px_#fff,0_0_5px_#08f,0_0_15px_#08f,0_0_30px_#08f]" />}
    </IconButton>;
}