import "./styles/styles.scss";
import "ui-neumorphism/dist/index.css";
import { Controller } from "./components/Controller";

export function App() {
    return <div className="h-[calc(100dvh)] bg-gray-900 flex items-center justify-center">
        <Controller />
    </div>;
}