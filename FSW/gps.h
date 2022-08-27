// Copyright 2022 akgvn

#pragma once

struct GPSData {
    double  latitude, longtitude, altitude; // altitude is in meters.
    uint8_t hour, minute, second;
    uint32_t satellites;

    String time() const;
};

void gps_setup();
bool get_gps_readings(GPSData& gps_data);
