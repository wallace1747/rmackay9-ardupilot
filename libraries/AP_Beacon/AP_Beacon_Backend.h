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
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include "AP_Beacon.h"

class AP_Beacon_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Beacon_Backend(AP_Beacon &frontend, AP_SerialManager &serial_manager);

    // update
    virtual void update() = 0;

    // set vehicle position
    void set_vehicle_position_ned(const Vector3f& pos, float accuracy_estimate);

    // set individual beacon distance in meters
    void set_beacon_distance(uint8_t beacon_instance, float distance);

protected:

    // references
    AP_Beacon &_frontend;
};
