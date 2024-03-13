import { Card } from "ui-neumorphism";

export interface DotMatrixScreenProps {
    text?: string;
    isError?: boolean;
}

export function DotMatrixScreen(props: DotMatrixScreenProps) {
    return <Card dark inset bordered className="font-lcd px-2 py-1 text-sky-200 h-9 flex justify-center items-center" style={{textShadow: "0 0 2px #08f"}}>
        <div className={props.isError ? "blink_me" : ""}>{props.text}</div>
    </Card>
}