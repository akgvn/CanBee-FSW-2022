// Copyright 2020 - 2022 akgvn

#include "sd_handler.h"

// include the SD library:
#include <SD.h>
#include <SPI.h>

File telemetryFile;

// Returns true on error.
bool sd_setup() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        // Uncomment the following for debugging.
        // Serial.println("SD begin error.");
        return true;
    }

    int file_count = 0;
    auto filename = String(file_count) + String(".csv");
    auto filename_c = filename.c_str();

    auto filename_found = false;
    while (!filename_found) {
        filename = String(file_count) + String(".csv");
        filename_c = filename.c_str();

        if (SD.exists(filename_c)) {
            file_count++;
        } else {
            filename_found = true;
            telemetryFile = SD.open(filename_c, FILE_WRITE);
        }
    }
    return false;
}

bool write_to_sd(const String& to_write) {
    auto results = telemetryFile.println(to_write);
    telemetryFile.flush(); // This is important.
    return false;
}