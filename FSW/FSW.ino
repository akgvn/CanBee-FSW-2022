// Copyright 2020 - 2022 akgvn

#include "configuration.h"

#include "gps.h"
#include "mpu9250.h"
#include "sd_handler.h"
#include "xbee_handler.h"
#include "cansat.h"
#include <Arduino.h>

#if IS_CONTAINER
Metro packet_timer = Metro(1000);
#else
Metro packet_timer = Metro(250);
#endif

bool error = false;

Cansat cansat;

void setup() {
    Serial.begin(9600);
    // while (!Serial) {}
    xbee_port.begin(9600); // Container or Payload, see #if above.

    xbee_port.setTimeout(10);
    while (!xbee_port) {}

    sd_setup();

    xbee_setup();

    error = cansat.setup();

#if IS_CONTAINER
    {
        // Pins to ground in the container
        pinMode(11, OUTPUT); digitalWrite(11, LOW);
        pinMode(24, OUTPUT); digitalWrite(24, LOW);
        pinMode(28, OUTPUT); digitalWrite(28, LOW);
    }
#endif

    while (error) Serial.println("Error in setup!");
}

double altitude_two_secs_ago = 0.0;
double altitude_one_sec_ago = 0.0;

int over_parachute_target_altitude_seconds = 0;
int over_payload_release_altitude_seconds = 0;

CommandType last_cmd = CommandType::None;

float previousTime = 0;
void loop() {
    // Container sensors: BMP280, Voltage, GPS, Xbee, Buzzer, Camera
    // Payload sensors: BMP280, MPU9250, Xbee, Camera

    // Read commands or data. Might be a GCS command (if container),
    // Payload telemetry (if container) or a Container command (if payload).

#if IS_PAYLOAD
    cansat.mpu_data.update();

    // Determining the elapsed time
    const float currentTime = micros(); // current time
    float deltaTimeSeconds = (currentTime - previousTime) / 1000000.0; //time difference in seconds
    previousTime = currentTime; //set the current time to get the next iteration difference

    cansat.gimbal.update(cansat.mpu_data.gyro.yaw, deltaTimeSeconds);
#endif

    if (packet_timer.check()) {
        auto err = cansat.update(altitude_one_sec_ago, altitude_two_secs_ago);
        if (err) Serial.println("Error in cansat.update");

        auto buffer = getZigBeeData();
        // Serial.print("Zigbee data: ");
        // Serial.println(buffer);

        auto cmd = strToCommand(buffer);

        if (cmd == CommandType::SimulatedPressureData) {
            auto subst = buffer.substring(4); // assuming SIMP101325
            cansat.lastSimPressure = (double) subst.toInt();
            Serial.print("subst :");
            Serial.println(subst);
            Serial.print("cansat.lastSP: ");
            Serial.println(cansat.lastSimPressure);
        }

        if (cmd != CommandType::None) {
            Serial.println(String("Command: ") + commandToString(cmd));
            cansat.executeCommand(cmd);
            last_cmd = cmd;
        }

        cansat.mission_time += 1;

        // Over parachute control
        {
            if (cansat.bmp.relative_altitude >= cansat.parachute_target_altitude)
                over_parachute_target_altitude_seconds += 1;
            else if ((cansat.bmp.relative_altitude < cansat.parachute_target_altitude) && (!cansat.over_parachute_target_altitude))
                over_parachute_target_altitude_seconds = 0;

            if (over_parachute_target_altitude_seconds > 3)
                cansat.over_parachute_target_altitude = true;
        }

        // Over payload control
        {
            if (cansat.bmp.relative_altitude >= cansat.payload_release_altitude)
                over_payload_release_altitude_seconds += 1;
            else if ((cansat.bmp.relative_altitude < cansat.payload_release_altitude) && (!cansat.over_payload_release_altitude))
                over_payload_release_altitude_seconds = 0;

            if (over_payload_release_altitude_seconds > 3)
                cansat.over_payload_release_altitude = true;
        }

#if IS_CONTAINER
        const auto telemetry = cansat.containerTelemetryStr(last_cmd);
#else
        const auto telemetry = cansat.payloadTelemetryStr();
#endif

        if (cansat.sendTelemetry && (telemetry.length() > 4))
        {
            cansat.packet_count += 1;
            Serial.print("Gonderilen telemetri: ");
            Serial.println(telemetry);
            sendData(telemetry, GROUND_CS_ADDRESS);
            write_to_sd(telemetry);
        }

        altitude_two_secs_ago = altitude_one_sec_ago;
        altitude_one_sec_ago  = cansat.bmp.relative_altitude;
    } // packet_timer.check()
}
