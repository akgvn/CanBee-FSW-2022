// Copyright 2020 - 2022 akgvn
// Some parts contributed by Berkay Apalı

#pragma once

#include <XBee.h>

// NOTE(AG): There used to be actual XBee addresses here,
// but let's not expose our XBee addresses publicly. 
static auto GROUND_CS_ADDRESS = XBeeAddress64(0x00000000, 0x00000000); // Type: Coordinator
static auto CONTAINER_ADDRESS = XBeeAddress64(0x00000000, 0x00000000); // Type: Router
static auto PAYLOAD_ADDRESS   = XBeeAddress64(0x00000000, 0x00000000); // Type: Router

void xbee_setup();
String getZigBeeData();
void sendData(const String &payload, XBeeAddress64 address);

enum class CommandType
{
    None,

    StartTelemetry,        StopTelemetry,
    StartPayloadTelemetry, StopPayloadTelemetry,
    StartCamera,           StopCamera,
    StartBuzzer,           StopBuzzer,
    StartSimulationMode,   StopSimulationMode,

    SimulatedPressureData,

    ReleaseParachute,
    ReleasePayload,
};

String commandToString(CommandType cmd);
CommandType strToCommand(const String &incoming_packet);
