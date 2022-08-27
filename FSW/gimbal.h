// Copyright 2022 akgvn & Samet Efekan Döşkaya

#pragma once

struct Gimbal
{
    static const int ENCODER_PIN_1 = 28; // This pin is also interrupt pin!
    static const int ENCODER_PIN_2 = 29;
    static const int PWMPinCW  = 18; // This pin assigned as positive direction pin
    static const int PWMPinCCW = 19; // This is an analog and negative direction pin pin to write the pwm signal-this pin can reach higher

    void setup();
    void update(float gyroYaw, float deltaTimeSeconds);

    float previousError = 0; // for calculating the derivative edot
private:
    
    float angle = 0;
    float oldGyroYaw = 0;

    // PID related time parameters
    float errorIntegral = 0; // integral error
};