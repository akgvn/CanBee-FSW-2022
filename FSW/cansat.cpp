// Copyright 2020 - 2022 akgvn

#include "cansat.h"
#include "Arduino.h"

static bool bmpMovingAverage(BMP &bmp) {
    double bmp_avg = 0.0;
    int sample_size = 5;
    bool err = false;
    for (int i = 0; i < sample_size; i++)
    {
        err = bmp.update();
        bmp_avg += bmp.relative_altitude;
    }
    bmp_avg /= sample_size;
    bmp.relative_altitude = bmp_avg;

    return err;
}

static bool isAscending(
    double current_altitude,
    double altitude_one_sec_ago,
    double altitude_two_secs_ago
) {
    return ((current_altitude - altitude_one_sec_ago) > 1.0)
        && ((altitude_one_sec_ago - altitude_two_secs_ago) > 1.0);
}

static bool isDescending(
    double current_altitude,
    double altitude_one_sec_ago,
    double altitude_two_secs_ago
) {
    return ((current_altitude - altitude_one_sec_ago) < -1.0)
        && ((altitude_one_sec_ago - altitude_two_secs_ago) < -1.0);
}


String Cansat::stateStr() const {
    switch (state) {
        case Cansat::FlightState::SensorsOff:       return String("SensorsOff");
        case Cansat::FlightState::BeforeLaunch:     return String("BeforeLaunch");
        case Cansat::FlightState::Ascent:           return String("Ascent");
        case Cansat::FlightState::Descent:          return String("Descent");
        case Cansat::FlightState::ReleaseParachute: return String("ReleaseParachute");
        case Cansat::FlightState::ReleasePayload:   return String("ReleasePayload");
        case Cansat::FlightState::Landing:          return String("Landing");
    }
    return String("Error in stateStr!");
}

static const String comma = String(",");

#if IS_CONTAINER
String Cansat::containerTelemetryStr(CommandType cmd) const {
    // <TEAM_ID>, <MISSION_TIME>, <PACKET_COUNT>, <PACKET_TYPE>, <MODE>,
    // <TP_RELEASED>, <ALTITUDE>, <TEMP>, <VOLTAGE>, <GPS_TIME>, <GPS_LATITUDE>,
    // <GPS_LONGITUDE>, <GPS_ALTITUDE>, <GPS_SATS>, <SOFTWARE_STATE>, <CMD_ECHO>

    auto pyld = (payload_released) ? String("1") : String("0");
    auto packet = String(team_id) +
        comma + String(mission_time) +
        comma + String(packet_count) +
        comma + String("C") +
        comma + String(modeStr()) +
        comma + String(pyld) +
        comma + String(altitude) +
        comma + String(temperature) +
        comma + String(voltage) +
        comma + String(gps.time()) +
        comma + String(gps.latitude) +
        comma + String(gps.longtitude) +
        comma + String(gps.altitude) +
        comma + String(gps.satellites) +
        comma + String(stateStr()) +
        comma + commandToString(cmd);

    return packet;
}
#endif

#if IS_PAYLOAD
String Cansat::payloadTelemetryStr() const {
    // <TEAM_ID>, <MISSION_TIME>, <PACKET_COUNT>, <PACKET_TYPE>, <TP_ALTITUDE>,
    // <TP_TEMP>, <TP_VOLTAGE>, <GYRO_R>, <GYRO_P>, <GYRO_Y>, <ACCEL_R>, <ACCEL_P>,
    // <ACCEL_Y>, <MAG_R>, <MAG_P>, <MAG_Y>, <POINTING_ERROR>, <TP_SOFTWARE_STATE>

    auto packet = String(team_id) +
        comma + String(mission_time) +
        comma + String(packet_count) +
        comma + String("T") +
        comma + String(altitude) +
        comma + String(temperature) +
        comma + String(voltage) +
        comma + String(mpu_data.gyro.roll) +
        comma + String(mpu_data.gyro.pitch) +
        comma + String(mpu_data.gyro.yaw) +
        comma + String(mpu_data.accelerometer.roll) +
        comma + String(mpu_data.accelerometer.pitch) +
        comma + String(mpu_data.accelerometer.yaw) +
        comma + String(mpu_data.magnetometer.roll) +
        comma + String(mpu_data.magnetometer.pitch) +
        comma + String(mpu_data.magnetometer.yaw) +
        comma + String(gimbal.previousError / 4.2) +
        comma + String(stateStr());

    return packet;
}
#endif

bool Cansat::setup() {
    state = Cansat::FlightState::SensorsOff;
    auto err = false;

    // gps_setup();

    mode = Mode::Flight;

#if IS_CONTAINER
    parachute_released = false;
    payload_released = false;
    type = PacketType::Container;
    payloadServo.setupServo(10, 180);
    payloadServo.servo_motor.write(130);
    
    // For payload 115-160
    parachuteServo.setupServo(12, 165);
    parachuteServo.servo_motor.write(75);

    setup_cam();
    disable_buzzer();
    #else
    type = PacketType::Payload;
    err = mpu_data.setup() || err;
#endif
    Serial.println("After MPU err is");
    Serial.println(err);

    
    voltmeter_setup();

#if IS_CONTAINER
    err = bmp.setup() || err;
    Serial.println("After BMP err is");
    Serial.println(err);
#endif

#if IS_PAYLOAD
    gimbal.setup();
#endif

    return err;
}

bool Cansat::update(double altitude_one_sec_ago, double altitude_two_secs_ago) {
    bool error = false;

    bool ascending  = isAscending(bmp.relative_altitude, altitude_one_sec_ago, altitude_two_secs_ago);
    bool descending = isDescending(bmp.relative_altitude, altitude_one_sec_ago, altitude_two_secs_ago);

    if (state > Cansat::FlightState::SensorsOff)
        sendTelemetry = true;

    if (ascending && state <= Cansat::FlightState::BeforeLaunch)
        state = Cansat::FlightState::Ascent;

    if (descending && state == Cansat::FlightState::Ascent)
        state = Cansat::FlightState::Descent;

    if (over_parachute_target_altitude && state == Cansat::FlightState::Descent && altitude <= parachute_target_altitude) {
        state = Cansat::FlightState::ReleaseParachute;
        
#if IS_CONTAINER
        parachute_released = true;
#endif

    }

    if (over_payload_release_altitude && state == Cansat::FlightState::ReleaseParachute && altitude <= payload_release_altitude) {
        state = Cansat::FlightState::ReleasePayload;
        
#if IS_CONTAINER
        payload_released = true;
        toggle_camera_recording();
#endif

    }

    if (parachute_released && payload_released && state == Cansat::FlightState::ReleasePayload && altitude <= 50)
        state = Cansat::FlightState::Landing;

#if IS_CONTAINER
    if (state == Cansat::FlightState::Landing) {
        enable_buzzer();
        toggle_camera_recording();
    }
#endif

    // BMP stuff
    // error = bmp.update();
#if IS_CONTAINER
    if (mode == Mode::Simulation)
        error = bmp.sim_update(lastSimPressure);
    else
        error = bmpMovingAverage(bmp);

    if (error) Serial.println("Error updating bmp");

    altitude = bmp.relative_altitude;
    temperature = bmp.temperature;
#endif

#if IS_CONTAINER
//    error = get_gps_readings(gps) || error;
    parachuteServo.update(parachute_released);
    payloadServo.update(payload_released);
#endif

    if (error) Serial.println("Error updating gps");

    voltage = get_voltage();

    return error;
}

#if IS_CONTAINER
void Cansat::setup_cam() {
    pinMode(CAM_TRIG_PIN, OUTPUT);
    digitalWrite(CAM_TRIG_PIN, HIGH);
}

// TODO Async wait
void Cansat::toggle_camera_recording() {
    digitalWrite(CAM_TRIG_PIN, LOW);
    delay(1000);
    digitalWrite(CAM_TRIG_PIN, HIGH);
}

void Cansat::CustomServo::setupServo(int servo_pin, int deploy) {
    Serial.println("SERVO STUFF IS HAPPENING");
    servo_motor.attach(servo_pin);
    angle = servo_motor.read();
    deploy_angle = deploy;
    Serial.print("Servo angle: " + angle);
    Serial.println(angle);
}

bool Cansat::CustomServo::update(bool should_deploy) {
    if (should_deploy) servo_motor.write(deploy_angle);
    
    return true;
}

#endif

float Cansat::get_voltage()
{
    const float R1 = 5700.0, R2 = 150000.0; // resistances of R1 and R2
    const float r1_r2_constant = ((R1 + R2) / R2);

    auto analog_value = analogRead(analogInputPin); // NOTE(ag): Bana gelen kodda float'a cast ediliyordu ama aslında analogRead int döndürüyor.
    float vout = (analog_value * 9.0) / 1024.0;     // hata alırsak bu satırdaki 9'a bakalım :D - Aydın
    float vin = vout * r1_r2_constant;

    return vin;
}

void Cansat::executeCommand(CommandType cmd) {
#if IS_PAYLOAD
    if (cmd == CommandType::StartPayloadTelemetry)
        sendTelemetry = true; // TODO for payload
    if (cmd == CommandType::StopPayloadTelemetry)
        sendTelemetry = false; // TODO for payload
#endif

#if IS_CONTAINER
    if (cmd == CommandType::StartCamera || cmd == CommandType::StopCamera)
        toggle_camera_recording();
    if (cmd == CommandType::StartBuzzer)
        enable_buzzer();
    if (cmd == CommandType::StopBuzzer)
        disable_buzzer();
    if (cmd == CommandType::ReleasePayload)
        payload_released = true;
    if (cmd == CommandType::ReleaseParachute)
        parachute_released = true;
    if (cmd == CommandType::StartTelemetry)
        sendTelemetry = true;
    if (cmd == CommandType::StopTelemetry)
        sendTelemetry = false;
    if (cmd == CommandType::StartSimulationMode) {
        mode = Mode::Simulation;
        bmp.baseline_pressure = sqrt(-1); // NaN
    }
    if (cmd == CommandType::StopSimulationMode)
        mode = Mode::Flight;

#endif
}

String Cansat::typeStr() const {
    if (type == Cansat::PacketType::Container) return String("C");
    if (type == Cansat::PacketType::Payload)   return String("T");
    return String("Error in typeStr");
}

String Cansat::modeStr() const {
    if (mode == Cansat::Mode::Flight)     return String("F");
    if (mode == Cansat::Mode::Simulation) return String("S");
    return String("Error in modeStr");
}
