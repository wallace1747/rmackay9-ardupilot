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

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Beacon_SITL.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define NUM_BEACONS 4
#define ORIGIN_DISTANCE 5

// constructor
AP_Beacon_SITL::AP_Beacon_SITL(AP_Beacon &frontend) :
    AP_Beacon_Backend(frontend)
{
    sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
}

// return true if sensor is basically healthy (we are receiving data)
bool AP_Beacon_SITL::healthy()
{
    // healthy if we have parsed a message within the past 300ms
    return ((AP_HAL::millis() - last_update_ms) < AP_BEACON_TIMEOUT_MS);
}

// update the state of the sensor
void AP_Beacon_SITL::update(void)
{
    uint8_t beacon_id = (next_beacon+1) % NUM_BEACONS;

    Location origin;
    origin.lat = get_beacon_origin_lat() * 1.0e7;
    origin.lng = get_beacon_origin_lon() * 1.0e7;
    origin.alt = get_beacon_origin_alt() * 1.0e2;

    Location beacon_loc = origin;
    switch (beacon_id) {
    case 0:
        location_offset(beacon_loc, ORIGIN_DISTANCE, -ORIGIN_DISTANCE);
        break;
    case 1:
        location_offset(beacon_loc, ORIGIN_DISTANCE, ORIGIN_DISTANCE);
        break;
    case 2:
        location_offset(beacon_loc, -ORIGIN_DISTANCE, ORIGIN_DISTANCE);
        break;
    case 3:
        location_offset(beacon_loc, -ORIGIN_DISTANCE, ORIGIN_DISTANCE);
        break;
    }

    Location current_loc;
    current_loc.lat = sitl->state.latitude * 1.0e7;
    current_loc.lng = sitl->state.longitude * 1.0e7;
    current_loc.alt = sitl->state.altitude * 1.0e2;

    Vector2f loc_diff  = location_diff(beacon_loc, current_loc);
    Vector2f beac_diff = location_diff(origin, beacon_loc);
    Vector2f veh_diff = location_diff(origin, current_loc);
    
    set_beacon_position(beacon_id, Vector3f(beac_diff.x, beac_diff.y, (beacon_loc.alt - origin.alt)*1.0e-2));
    set_beacon_distance(beacon_id, loc_diff.length());
    set_vehicle_position_ned(Vector3f(veh_diff.x, veh_diff.y, (current_loc.alt - origin.alt)*1.0e-2), 0.5f);
    last_update_ms = AP_HAL::millis();
}

#endif // CONFIG_HAL_BOARD
