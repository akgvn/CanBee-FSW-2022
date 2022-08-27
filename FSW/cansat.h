// Copyright 2020 - 2022 akgvn

#pragma once

#include <Servo.h>
#include <Metro.h>
#include "configuration.h"
#include "MPU9250.h"
#include "GPS.h"
#include "xbee_handler.h"
#include "bmp280.h"
#include "gimbal.h"

struct Cansat {
    enum class Mode {
        Flight     = 'F',
        Simulation = 'S'
    };

    enum class PacketType {
        Container = 'C',
        Payload   = 'T',
        Custom    = 'X'
    };

    enum class FlightState {
        SensorsOff,
        BeforeLaunch,
        Ascent,
        Descent,
        ReleaseParachute,
        ReleasePayload,
        Landing,
    };

    struct CustomServo {
        Servo servo_motor;
        int angle;
        int deploy_angle;
        Metro servo_timer;

        CustomServo(): servo_timer(Metro(0)) {}

        const byte SERVO_PIN = 6;

        void setupServo(int servo_pin, int deploy_angle);
        bool update(bool should_deploy);
    };

    const unsigned int team_id = TEAM_ID;
    unsigned int mission_time;
    unsigned int packet_count;
    PacketType type;
    FlightState state;
    float altitude;
    float temperature;
    float voltage;
    bool sendTelemetry = false;
    Mode mode;
    double lastSimPressure = sqrt(-1); // NaN

#if IS_CONTAINER
    // Only for Container
    GPSData gps;
    
    CustomServo parachuteServo;
    CustomServo payloadServo;

    String containerTelemetryStr(CommandType cmd) const;

    const byte CAM_TRIG_PIN = 25; // Container
    void setup_cam();
    void toggle_camera_recording();

    const byte BUZZER_PIN = 7;
    void enable_buzzer()  { pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, HIGH); }
    void disable_buzzer() { pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);  }

    const byte analogInputPin = 39; // Container 39. pin

#else
    // Only for Payload
    Gimbal gimbal;
    MPU mpu_data;
    float pointing_error;
    
    String payloadTelemetryStr() const;
    const byte analogInputPin = 23; // Payload 23. pin
    
#endif

    BMP bmp;
    bool parachute_released;
    bool payload_released;
    bool over_parachute_target_altitude = false;
    bool over_payload_release_altitude = false;
    static const int parachute_target_altitude = 400; // in meters
    static const int payload_release_altitude = 300;  // in meters

    String stateStr() const;
    String typeStr() const;
    String modeStr() const;

    bool setup();
    bool update(double altitude_one_sec_ago, double altitude_two_secs_ago);
    void executeCommand(CommandType cmd);

    void voltmeter_setup() { pinMode(analogInputPin, INPUT); }
    float get_voltage();
};
