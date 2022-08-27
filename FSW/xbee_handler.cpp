// Copyright 2020 - 2022 akgvn
// Some parts contributed by Berkay Apalı

#include "configuration.h"
#include "xbee_handler.h"

// I don't like this - ag
XBee xbee = XBee();

void xbee_setup() {
    xbee.begin(xbee_port);
}

String getZigBeeData() {
    xbee.readPacket();
    auto response = xbee.getResponse();

    if (response.isAvailable() && response.getApiId() == ZB_RX_RESPONSE) {
        auto rx = ZBRxResponse();
        xbee.getResponse().getZBRxResponse(rx);

        uint8_t len   = rx.getDataLength();
        uint8_t* data = rx.getData(); // This may be accessed from index 0 to getDataLength() - 1

        auto resultString = String("");

        for (size_t idx = 0; idx < len; idx++) {
            auto character_uint = data[idx];
            if (character_uint != 0)
                resultString += (char)character_uint;
            else
                break;
        }

        return resultString;
    }
    return String("");
}

void sendData(const String &payload, XBeeAddress64 address) {
    // TODO const char* to uint8_t* conversion warning!
    const uint8_t* raw_payload = (uint8_t*) payload.c_str();
    const uint8_t  payload_length = payload.length();

    auto txReq = ZBTxRequest(address, raw_payload, payload_length);
    xbee.send(txReq);
}

// Command stuff
// The following is why it is not fun to use C++'s enum classes.

String commandToString(CommandType cmd) {
    switch (cmd) {
        case CommandType::None:                  return String("None");
        case CommandType::StartTelemetry:        return String("StartTelemetry");
        case CommandType::StopTelemetry:         return String("StopTelemetry");
        case CommandType::StartPayloadTelemetry: return String("StartPayloadTelemetry");
        case CommandType::StopPayloadTelemetry:  return String("StopPayloadTelemetry");
        case CommandType::StartSimulationMode:   return String("StartSimulationMode");
        case CommandType::StopSimulationMode:    return String("StopSimulationMode");
        case CommandType::SimulatedPressureData: return String("SimulatedPressureData");
        case CommandType::StartCamera:           return String("StartCamera");
        case CommandType::StopCamera:            return String("StopCamera");
        case CommandType::StartBuzzer:           return String("StartBuzzer");
        case CommandType::StopBuzzer:            return String("StopBuzzer");
        case CommandType::ReleaseParachute:      return String("ReleaseParachute");
        case CommandType::ReleasePayload:        return String("ReleasePayload");
    }
    return String("Error in commandToString!");
}

CommandType strToCommand(const String &incoming_packet)
{
    if (incoming_packet.indexOf("CX ON")   != -1) return CommandType::StartTelemetry;
    if (incoming_packet.indexOf("CX OFF")  != -1) return CommandType::StopTelemetry;

    if (incoming_packet.indexOf("TP ON")   != -1) return CommandType::StartPayloadTelemetry;
    if (incoming_packet.indexOf("TP OFF")  != -1) return CommandType::StopPayloadTelemetry;

    if (incoming_packet.indexOf("PRCHUTE") != -1) return CommandType::ReleaseParachute;
    if (incoming_packet.indexOf("RELEASE") != -1) return CommandType::ReleasePayload;

    if (incoming_packet.indexOf("CAM ON")  != -1) return CommandType::StartCamera;
    if (incoming_packet.indexOf("CAM OFF") != -1) return CommandType::StopCamera;

    if (incoming_packet.indexOf("BUZ ON")  != -1) return CommandType::StartBuzzer;
    if (incoming_packet.indexOf("BUZ OFF") != -1) return CommandType::StopBuzzer;

    if (incoming_packet.indexOf("SIM ON")  != -1) return CommandType::StartSimulationMode;
    if (incoming_packet.indexOf("SIM OFF") != -1) return CommandType::StopSimulationMode;

    if (incoming_packet.indexOf("SIMP") != -1)    return CommandType::SimulatedPressureData;
    
    return CommandType::None;
}
