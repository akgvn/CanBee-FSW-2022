// Copyright 2022 akgvn & Burak Geçer

#pragma once

#include <MPU9250_asukiaaa.h>
struct MPU {
    struct MPUReading {
        float roll, pitch, yaw;
    };

    MPUReading gyro;
    MPUReading accelerometer;
    MPUReading magnetometer;

    bool setup();
    bool update();
    
    MPU9250_asukiaaa rawSensor;
};
