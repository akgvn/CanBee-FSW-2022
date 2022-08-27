// Copyright 2020 - 2022 akgvn

#pragma once

#define IS_CONTAINER true
#define IS_PAYLOAD (!(IS_CONTAINER))

#define XBEE_API_MODE true
#define SIM_MODE false

#define xbee_port Serial8

#define TEAM_ID 1040

#include "Arduino.h"
#include "cansat.h"

// Timing library that is included in Teensy.
// See: https://www.pjrc.com/teensy/td_libs_Metro.html
#include <Metro.h>
