export interface DotMatrixScreenProps {
    text?: string;
    isError?: boolean;
}

export function DotMatrixScreen(props: DotMatrixScreenProps) {
    return <div className="bg-gray-950 font-lcd rounded-md px-2 py-1 border border-gray-700 text-sky-200 h-9 flex justify-center items-center" style={{textShadow: "0 0 2px #08f"}}>
        <div className={props.isError ? "blink_me" : ""}>{props.text}</div>
    </div>
}