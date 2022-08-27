// Copyright 2022 akgvn & Samet Efekan Döşkaya
// PID Algorithm for the gimbal camera stabilization thing

#include "gimbal.h"

#include <Arduino.h>

struct PIDResult {
    float controlSignal = 0;
    float errorValue = 0;
};

static volatile float MOTOR_POSITION = 0; // position based on the encoder
static void driveMotor(float motorPosition, float targetValue);
static void checkEncoderAndUpdateMotorPosition();
static PIDResult calculatePID(
    float  motorPosition,
    float  targetValue,
    float  deltaTimeInSeconds,
    float& previousError,
    float& errorIntegral
);

void Gimbal::setup() {
    // Motor encoder-related
    pinMode(Gimbal::ENCODER_PIN_1, INPUT); // A
    pinMode(Gimbal::ENCODER_PIN_2, INPUT);
    pinMode(Gimbal::PWMPinCW, OUTPUT);
    pinMode(Gimbal::PWMPinCCW, OUTPUT);    // B
    attachInterrupt(
        digitalPinToInterrupt(Gimbal::ENCODER_PIN_1),
        checkEncoderAndUpdateMotorPosition,
        RISING
    );

    // Wire1.begin();
    // mySensor.setWire(&Wire1);
    // mySensor.beginGyro();
}

const float offset = -0.87; // for gyro yaw
void Gimbal::update(float gyroYaw, float deltaTimeSeconds) {
    gyroYaw += offset;
    if(gyroYaw >= -0.2 && gyroYaw <= 0.2)
        gyroYaw = 0;
    
    angle += (oldGyroYaw + gyroYaw) * 0.5 * deltaTimeSeconds;

    float targetValue = -angle*3.17;

    auto pidResult = calculatePID(MOTOR_POSITION, targetValue, deltaTimeSeconds, previousError, errorIntegral);
    driveMotor(pidResult.controlSignal, pidResult.errorValue);

    oldGyroYaw = gyroYaw;
}

void driveMotor(float controlSignal, float errorValue) {
    // Determine speed and direction of the motor based on the value of the control signal
    // Direction also needs to be computed

    // PWMValue is is the pin that stores the PWM value to be sent to pin 11
    int PWMValue = (int) fabs(controlSignal); // PWM values can not be negative

    if (PWMValue >= 255) // fabs() = floating point absolute value
        PWMValue = 255; // capping the PWM signal - 8 bit

    if (PWMValue <30 && errorValue != 0)
        PWMValue = 30; // Will be adjusted
  
    if (controlSignal > 0)
        analogWrite(Gimbal::PWMPinCW, PWMValue);
    else if (controlSignal < 0)
        analogWrite(Gimbal::PWMPinCCW, PWMValue);
}

PIDResult calculatePID(
    float  motorPosition,
    float  targetValue,
    float  deltaTimeInSeconds,
    float& previousError,
    float& errorIntegral
) {
    // PID related constants
    const float proportional = 1.0;     // k_p value
    const float integral     = 0.0085;  // k_i value
    const float derivative   = 0.085;   // k_d value

    const float errorValue = motorPosition - targetValue; // current position - target position
    
    const float edot = (errorValue - previousError) / deltaTimeInSeconds; // edot = de/dt derivative term
    previousError = errorValue; // save the error for the next iteration to get the difference

    errorIntegral += (errorValue + previousError) * 0.5 * deltaTimeInSeconds; // running sum

    // controlSignal is u - Also called as process variable(PV)
    float controlSignal = (proportional * errorValue) + (derivative * edot) + (integral * errorIntegral);

    PIDResult rt;
    rt.controlSignal = controlSignal;
    rt.errorValue = errorValue;
   
    return rt;
}

void checkEncoderAndUpdateMotorPosition() {
    // This function is called when an interrupt happens (see attachInterrupt line in the setup function)
    const auto motorDirection = digitalRead(Gimbal::ENCODER_PIN_2);

    // Value of the encoder pin can be 0 or 1 (since we are doing a digitalRead).
    // 1 means clockwise direction and 0 means counterclockwise direction.
    if (motorDirection == 1) MOTOR_POSITION++; // CW direction
    if (motorDirection == 0) MOTOR_POSITION--; // CCW direction
}
