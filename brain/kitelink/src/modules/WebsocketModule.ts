import { KitelinkModuleBase } from "./KitelinkModuleBase";
import { WebSocketServer, WebSocket } from "ws";
import { MavLinkPacket } from "node-mavlink";
import { IncomingMessage } from "http";
import { Utils } from "../Utils";

interface WebsocketWithConnection extends WebSocket {
    request: IncomingMessage;
}

interface MAVLinkJSONMessageIn {
    mav_type: string;

    [name: string]: any;
}

interface MAVLinkJSONMessageOut extends MAVLinkJSONMessageIn {
    mav_header?: { sys: number, comp: number };
}

export class WebsocketModule extends KitelinkModuleBase {

    private _server?: WebSocketServer;

    private static toJSON(data: any) {
        return JSON.stringify(data, (key, value) => {
            if (typeof value === "bigint") {
                return Number(value.toString());
            }

            return value;
        });
    }

    async handleIncomingJSONMessage(data: string) {
        const parsedData: MAVLinkJSONMessageIn = JSON.parse(data);

        const msgName = parsedData.mav_type;

        if (!msgName) {
            return;
        }

        const clazz = this.getMessageClassFromName(msgName);
        if (!clazz) {
            return;
        }

        const msgInstance = new clazz();

        clazz.FIELDS.forEach((field) => {
            (msgInstance as any)[field.name] = parsedData[field.source];
        });

        await this.sendMessage(msgInstance);
    }

    onLoad() {
        this._server = new WebSocketServer({ port: Number(process.env.WS_PORT ?? 3000) });

        this._server.on("connection", (connection: WebsocketWithConnection, request) => {

            connection.request = request;

            connection.on("error", console.error);
            connection.on("message", async (data) => {
                try {
                    if (request.url === "/json") {
                        await this.handleIncomingJSONMessage(data.toString());
                    }
                } catch (e) {
                    console.error(e);
                }
            });
        });
    }

    onUnload() {
        if (this._server) {
            this._server.close();
            this._server = undefined;
        }
    }

    onReceivePacket(packet: MavLinkPacket) {
        if (!this._server) {
            return;
        }

        const jsonObject = this.packetToJSONObject(packet);

        this._server.clients.forEach(client => {
            const connection = client as WebsocketWithConnection;
            if (connection.readyState === WebSocket.OPEN) {
                if (jsonObject && connection.request.url === "/json") {
                    client.send(WebsocketModule.toJSON(jsonObject));
                }
            }
        });
    }

    packetToJSONObject(packet: MavLinkPacket): MAVLinkJSONMessageOut | undefined {
        const packetClass = this.getMessageClassFromPacket(packet);

        if (!packetClass) {
            return;
        }

        const parsedMessage = Utils.parseMAVLinkPacket(packet, this._options.registry);

        if (!parsedMessage) {
            return;
        }

        const jsonObject: MAVLinkJSONMessageOut = {
            mav_type: packetClass.MSG_NAME,
            mav_header: {
                sys: packet.header.sysid,
                comp: packet.header.compid
            }
        };

        packetClass.FIELDS.forEach(field => {
            jsonObject[field.source] = (parsedMessage as any)[field.name];
        });

        return jsonObject;
    }
}