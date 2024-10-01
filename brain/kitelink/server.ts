import { connect } from "net";
import { KitelinkModuleBase } from "./src/modules/KitelinkModuleBase";
import { NTRIPModule } from "./src/modules/NTRIPModule";
import { KitelinkServer } from "./src/KitelinkServer";
import { WebsocketModule } from "./src/modules/WebsocketModule";

const modules: typeof KitelinkModuleBase[] = [
    NTRIPModule,
    WebsocketModule
];

const port = connect({ host: process.env.MAV_TCP_HOST ?? "lawnny.local", port: Number(process.env.MAV_TCP_PORT ?? 5760) });

const server = new KitelinkServer({
    port,
    modules
});
server.start();

// process.on("exit", () => {
//     server.stop();
// });