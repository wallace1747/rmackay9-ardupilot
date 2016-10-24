// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#pragma once

#include "AP_Beacon_Backend.h"

class AP_Beacon_Pozyx : public AP_Beacon_Backend
{

public:
    // constructor
    AP_Beacon_Pozyx(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // update
    void update();

private:

    AP_HAL::UARTDriver *uart = nullptr;
    char linebuf[10];
    uint8_t linebuf_len = 0;
};
