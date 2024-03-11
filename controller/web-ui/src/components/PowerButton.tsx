export interface PowerButtonProps {
    on: boolean;
    onClick?: () => void;
}

export function PowerButton(props: PowerButtonProps) {
    return <div onClick={props.onClick}
                className={`cursor-pointer bg-gradient-to-b from-gray-900 to-gray-800 active:bg-gradient-to-t inline-block p-3 rounded-full border-4 ${props.on ? "border-sky-200 shadow-[0_0_2px_#fff,inset_0_0_2px_#fff,0_0_5px_#08f,0_0_15px_#08f,0_0_30px_#08f] bg-sky-200" : "border-gray-700"}`}>
        <svg xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24" strokeWidth={1.5} stroke="currentColor" className="w-8 h-8">
            <path strokeLinecap="round" strokeLinejoin="round" d="M5.636 5.636a9 9 0 1 0 12.728 0M12 3v9"/>
        </svg>
    </div>
}