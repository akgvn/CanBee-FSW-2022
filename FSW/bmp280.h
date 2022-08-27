// Copyright 2022 akgvn & Burak Geçer

#pragma once

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

struct BMP {
    double temperature, relative_altitude, baseline_pressure;
    unsigned status;

    // The wire depends on the PCB!!!
    Adafruit_BMP280 bmp = Adafruit_BMP280(&Wire); // I2C

    bool setup();
    bool update();
    bool sim_update(double pressure);
};
