// Copyright 2022 akgvn

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "gps.h"

static const int RXPin = 0;
static const int TXPin = 1;
static const int GPSBaud = 9600;

static TinyGPSPlus tgps; // Create a TinyGPS++ object
static SoftwareSerial gpsSerial(RXPin, TXPin);

void gps_setup() {
    gpsSerial.begin(GPSBaud);
}

bool get_gps_readings(GPSData& gps_data) {
    bool error = false; 

    if (tgps.location.isValid()) {
        gps_data.latitude   = tgps.location.lat(); // Latitude ->  (double)
        gps_data.longtitude = tgps.location.lng(); // Longitude -> (double)
        gps_data.altitude   = tgps.altitude.meters(); // in meters, double
    } else { error = true; }

    if (tgps.time.isValid()) {
        gps_data.hour   = tgps.time.hour();
        gps_data.minute = tgps.time.minute();
        gps_data.second = tgps.time.second();
    } else { error = true; }

    if (tgps.satellites.isValid()) {
        gps_data.satellites = tgps.satellites.value();
    } else { error = true; }

    return error;
}

String GPSData::time() const {
    return String(hour) + ":" + String(minute) + ":" + String(second);
}