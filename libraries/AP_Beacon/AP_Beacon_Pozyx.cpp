// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_Beacon_Pozyx.h"
#include <ctype.h>

extern const AP_HAL::HAL& hal;

// constructor
AP_Beacon_Pozyx::AP_Beacon_Pozyx(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    AP_Beacon_Backend(frontend, serial_manager),
    linebuf_len(0)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Lidar, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_Lidar, 0));
    }
}

// update the state of the sensor
void AP_Beacon_Pozyx::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // read any available lines from the lidar
    float sum = 0;
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
        char c = uart->read();
        if (c == '\r') {
            linebuf[linebuf_len] = 0;
            sum += (float)atof(linebuf);
            count++;
            linebuf_len = 0;
        } else if (isdigit(c) || c == '.') {
            linebuf[linebuf_len++] = c;
            if (linebuf_len == sizeof(linebuf)) {
                // too long, discard the line
                linebuf_len = 0;
            }
        }
    }

    // we need to write a byte to prompt another reading
    uart->write('d');
}

