import { MavLinkData, MavLinkDataConstructor, MavLinkPacket, MavLinkPacketRegistry } from "node-mavlink";

export class Utils {
    static parseMAVLinkPacket<T extends MavLinkData = MavLinkData>(packet: MavLinkPacket, registry: MavLinkPacketRegistry): T | undefined {
        const clazz = registry[packet.header.msgid];
        if (clazz) {
            return packet.protocol.data(packet.payload, clazz) as T;
        }
    }

    static splitBuffer(buffer: Buffer, chunkSize: number): Buffer[] {
        const chunks: Buffer[] = [];
        for (let i = 0; i < buffer.length; i += chunkSize) {
            chunks.push(buffer.subarray(i, i + chunkSize));
        }
        return chunks;
    }
}