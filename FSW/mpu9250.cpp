// Copyright 2022 akgvn & Burak Geçer

#include "mpu9250.h"

bool MPU::setup() {
    // Note that The specific wire used depends on your PCB design.
    // The MPU9250_asukiaaa we used here might also need some adjustments.
    Wire1.begin();
    rawSensor.setWire(&Wire1);

    rawSensor.beginAccel();
    rawSensor.beginGyro();
    
    // Magnetometer fails for no reason, but we don't need it anyway.
    // Using Samet's PID method for the gimbal works flawlessly.
    rawSensor.beginMag();

    return false;
}

bool MPU::update() {
    if (rawSensor.accelUpdate() == 0) {
        accelerometer.roll  = rawSensor.accelX();
        accelerometer.pitch = rawSensor.accelY();
        accelerometer.yaw   = rawSensor.accelZ();
    }

    if (rawSensor.gyroUpdate() == 0) {
        gyro.roll  = rawSensor.gyroX();
        gyro.pitch = rawSensor.gyroY();
        gyro.yaw   = rawSensor.gyroZ();
    }

    if (rawSensor.magUpdate() == 0) {
        magnetometer.roll  = rawSensor.magX();
        magnetometer.pitch = rawSensor.magY();
        magnetometer.yaw   = rawSensor.magZ();
    }

    return false;
}
