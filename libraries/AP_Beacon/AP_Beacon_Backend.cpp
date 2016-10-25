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

#include "AP_Beacon_Backend.h"
// debug
#include <stdio.h>

/*
  base class constructor. 
  This incorporates initialisation as well.
*/
AP_Beacon_Backend::AP_Beacon_Backend(AP_Beacon &frontend, AP_SerialManager &serial_manager) :
    _frontend(frontend)
{
}

// set vehicle position
void AP_Beacon_Backend::set_vehicle_position_ned(const Vector3f& pos, float accuracy_estimate)
{
    _frontend.veh_pos_update_ms = AP_HAL::millis();
    _frontend.veh_pos_ned = pos;
    _frontend.veh_pos_accuracy = accuracy_estimate;

    // debug
    ::printf("vehicle x:%4.2f y:%4.2f z:%4.2f error:%4.2f\n", (double)pos.x, (double)pos.y, (double)pos.z, (double)accuracy_estimate);
}

// set individual beacon distance in meters
void AP_Beacon_Backend::set_beacon_distance(uint8_t beacon_instance, float distance)
{
    // sanity check instance
    if (beacon_instance > AP_BEACON_MAX_BEACONS) {
        return;
    }

    // setup new beacon
    if (beacon_instance > _frontend.num_beacons) {
        _frontend.num_beacons = beacon_instance;
    }

    _frontend.beacon_state[beacon_instance].distance_update_ms = AP_HAL::millis();
    _frontend.beacon_state[beacon_instance].distance = distance;
    _frontend.beacon_state[beacon_instance].healthy = true;

    ::printf("beacon %d dist:%4.2f\n", (int)beacon_instance, (double)distance);
}

// configure beacon's position in meters from origin
void AP_Beacon_Backend::set_beacon_position(uint8_t beacon_instance, const Vector3f& pos)
{
    // sanity check instance
    if (beacon_instance > AP_BEACON_MAX_BEACONS) {
        return;
    }

    // setup new beacon
    if (beacon_instance > _frontend.num_beacons) {
        _frontend.num_beacons = beacon_instance;
    }

    ::printf("beacon %d x:%4.2f y:%4.2f z:%4.2f\n", (int)beacon_instance, (double)pos.x, (double)pos.y, (double)pos.z);

    // set position
    _frontend.beacon_state[beacon_instance].position = pos;
}
