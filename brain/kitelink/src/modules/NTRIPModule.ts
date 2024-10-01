import { KitelinkModuleBase } from "./KitelinkModuleBase";
import { MavLinkPacket, common } from "node-mavlink";
// @ts-ignore
import { NtripClient } from "ntrip-client";
import { Utils } from "../Utils";

const { geoToEcef } = require("ntrip-client/lib/nmea/ecef");

export class NTRIPModule extends KitelinkModuleBase {

    _ntripClient?: NtripClient;
    _sequenceNumber: number = 0;

    onLoad() {
        this.onUnload();

        this._ntripClient = new NtripClient({
            host: process.env.NTRIP_HOST ?? "rtkbase.local",
            port: Number(process.env.NTRIP_PORT ?? 2101),
            mountpoint: process.env.NTRIP_MOUNTPOINT ?? "home",
            username: process.env.NTRIP_USERNAME ?? "lawnny",
            password: process.env.NTRIP_PASSWORD ?? "NeedMowInput",
            xyz: [0, 0, 0],
            interval: 2000
        });

        this._ntripClient.on("data", async (data: any) => {
            await this.sendRTCMMessage(data);
        });

        this._ntripClient.on("close", () => {
            console.log("NTRIP Module Connection Closed");
        });

        this._ntripClient.on("error", (err: any) => {
            console.error("NTRIP Module Connection Error:", err);
        });

        this._ntripClient.run();
    }

    onUnload() {
        if (this._ntripClient) {
            this._ntripClient.close();
            this._ntripClient = undefined;
        }
    }

    onReceivePacket(packet: MavLinkPacket) {

        if (!this._ntripClient) {
            return;
        }

        const packetType = this.getPacketMessageType(packet);
        if (packetType === "GPS_RAW_INT") {
            const gpsRawInt = this.parseMAVLinkPacket<common.GpsRawInt>(packet)!;

            // Convert our GEO coordinates to ECEF
            if (gpsRawInt.fixType >= 2) {
                const ecefCoords = geoToEcef([gpsRawInt.lat / 1E7, gpsRawInt.lon / 1E7, gpsRawInt.alt / 1E3]);
                this._ntripClient.setXYZ(ecefCoords);
            }
        }
    }

    async sendRTCMMessage(data: Buffer) {
        let flags = 0;
        if (data.length > 180) {
            flags = 1;
        }

        // add in the sequence number
        flags |= (this._sequenceNumber & 0x1F) << 3;

        this._sequenceNumber++;

        if (data.length > 4 * 180) {
            console.error("Unable to send RTCM data: data too long");
            return;
        }

        // Split the Buffer into 180 byte chunks
        const chunks = Utils.splitBuffer(data, 180);

        for (let i = 0; i < chunks.length; i++) {
            const chunk = chunks[i];
            const msg = new common.GpsRtcmData();
            msg.flags = flags | (i << 1);
            msg.len = chunk.length;
            // @ts-ignore
            msg.data = chunk;
            await this.sendMessage(msg);
        }
    }
}