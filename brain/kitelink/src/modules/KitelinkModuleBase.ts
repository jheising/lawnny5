import { MavLinkProtocolV2, send, MavLinkData, MavLinkPacketHeader, MavLinkPacket, MavLinkPacketRegistry, MavLinkDataConstructor } from "node-mavlink";
import { Stream } from "stream";
import { Utils } from "../Utils";

export interface KitelinkModuleOptions {
    port: Stream;
    registry: MavLinkPacketRegistry;
}

export abstract class KitelinkModuleBase {
    protected readonly _options: KitelinkModuleOptions;

    constructor(options: KitelinkModuleOptions) {
        this._options = options;
    }

    onLoad() {
    }

    onUnload() {
    }

    onReceivePacket?(packet: MavLinkPacket): void;

    async sendMessage(msg: MavLinkData) {
        await send(this._options.port as any, msg, new MavLinkProtocolV2());
    }

    parseMAVLinkPacket<T extends MavLinkData = MavLinkData>(packet: MavLinkPacket):T | undefined {
        return Utils.parseMAVLinkPacket(packet, this._options.registry);
    }

    getMessageClassFromName(msgName: string): MavLinkDataConstructor<MavLinkData> | undefined {
        return Object.values(this._options.registry).find(value => value.MSG_NAME === msgName);
    }

    getPacketMessageType(packet: MavLinkPacket): string | undefined {
        return this.getMessageClassFromPacket(packet)?.MSG_NAME;
    }

    getMessageClassFromPacket(packet: MavLinkPacket): MavLinkDataConstructor<MavLinkData> | undefined {
        return this._options.registry[packet.header.msgid];
    }
}