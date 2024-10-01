import { KitelinkModuleBase } from "./modules/KitelinkModuleBase";
import { Stream } from "stream";
import {
    MavLinkPacketParser, MavLinkPacketSplitter, MavLinkPacketRegistry,
    minimal, common, ardupilotmega, MavLinkPacket, MavLinkProtocolV2
} from "node-mavlink";

const REGISTRY: MavLinkPacketRegistry = {
    ...minimal.REGISTRY,
    ...common.REGISTRY,
    ...ardupilotmega.REGISTRY
};

export interface KitelinkServerOptions {
    port: Stream;
    modules: (typeof KitelinkModuleBase)[];
}

export class KitelinkServer {
    private readonly _options: KitelinkServerOptions;
    private _modules: KitelinkModuleBase[] = [];

    constructor(options: KitelinkServerOptions) {
        this._options = options;

        const reader = this._options.port
            .pipe(new MavLinkPacketSplitter())
            .pipe(new MavLinkPacketParser());

        reader.on("data", (packet) => this._onPacketReceived(packet));
    }

    start() {
        // @ts-ignore
        this._modules = this._options.modules.map(ModuleType => new ModuleType({
            port: this._options.port,
            registry: REGISTRY
        }));

        for (let module of this._modules) {
            module.onLoad();
        }
    }

    stop() {
        for (let module of this._modules) {
            module.onUnload();
        }

        this._modules = [];
    }

    private _onPacketReceived(packet: MavLinkPacket) {
        if (this._modules.length === 0) {
            return;
        }

        for (let module of this._modules) {
            if (module.onReceivePacket) {
                module.onReceivePacket(packet);
            }
        }
    }
}