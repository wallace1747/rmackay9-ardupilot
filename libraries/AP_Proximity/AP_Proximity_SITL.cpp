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
#include <AP_Param/AP_Param.h>
#include "AP_Proximity_SITL.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define PROXIMITY_MAX_RANGE 200
#define PROXIMITY_ACCURACY 0.1

/* 
   The constructor also initialises the proximity sensor. 
*/
AP_Proximity_SITL::AP_Proximity_SITL(AP_Proximity &_frontend,
                                     AP_Proximity::Proximity_State &_state):
    AP_Proximity_Backend(_frontend, _state)
{
    sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    ap_var_type ptype;
    fence_count = (AP_Int8 *)AP_Param::find("FENCE_TOTAL", &ptype);
    if (fence_count == nullptr || ptype != AP_PARAM_INT8) {
        AP_HAL::panic("Proximity_SITL: Failed to find FENCE_TOTAL");
    }
}

// get distance in meters in a particular direction in degrees (0 is forward, angles increase in the clockwise direction)
bool AP_Proximity_SITL::get_horizontal_distance(float angle_deg, float &distance) const
{
    if (!fence_loader.boundary_valid(fence_count->get(), fence, true)) {
        return false;
    }

    // convert to earth frame
    angle_deg = wrap_360(sitl->state.yawDeg + angle_deg);

    /*
      simple bisection search to find distance. Not really efficient,
      but we can afford the CPU in SITL
     */
    float min_dist = 0, max_dist = PROXIMITY_MAX_RANGE;
    while (max_dist - min_dist > PROXIMITY_ACCURACY) {
        float test_dist = (max_dist+min_dist)*0.5f;
        Location loc = current_loc;
        location_update(loc, angle_deg, test_dist);
        Vector2l vecloc(loc.lat, loc.lng);
        if (fence_loader.boundary_breached(vecloc, fence_count->get(), fence, true)) {
            max_dist = test_dist;
        } else {
            min_dist = test_dist;
        }
    }
    distance = min_dist;
    return true;
}

// update the state of the sensor
void AP_Proximity_SITL::update(void)
{
    load_fence();
    current_loc.lat = sitl->state.latitude * 1.0e7;
    current_loc.lng = sitl->state.longitude * 1.0e7;
    current_loc.alt = sitl->state.altitude * 1.0e2;
    if (fence && fence_loader.boundary_valid(fence_count->get(), fence, true)) {
        set_status(AP_Proximity::Proximity_Good);
        // update distance in one sector
        get_horizontal_distance(last_sector * 45, _distance[last_sector]);
        update_boundary_for_sector(last_sector);
        last_sector++;
        if (last_sector >= 8) {
            last_sector = 0;
        }
    } else {
        set_status(AP_Proximity::Proximity_NoData);        
    }
}

void AP_Proximity_SITL::load_fence(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_load_ms < 1000) {
        return;
    }
    last_load_ms = now;
    
    if (fence == nullptr) {
        fence = (Vector2l *)fence_loader.create_point_array(sizeof(Vector2l));
    }
    if (fence == nullptr) {
        return;
    }
    for (uint8_t i=0; i<fence_count->get(); i++) {
        fence_loader.load_point_from_eeprom(i, fence[i]);
    }
}

// get distance and angle to closest object (used for pre-arm check)
//   returns true on success, false if no valid readings
bool AP_Proximity_SITL::get_closest_object(float& angle_deg, float &distance) const
{
    bool sector_found = false;
    uint8_t sector = 0;

    // check all sectors for shorter distance
    for (uint8_t i=0; i<8; i++) {
        if (!sector_found || (_distance[i] < _distance[sector])) {
            sector = i;
            sector_found = true;
        }
    }

    if (sector_found) {
        angle_deg = sector * 45;
        distance = _distance[sector];
    }
    return sector_found;
}

// get boundary points around vehicle for use by avoidance
//   returns nullptr and sets num_points to zero if no boundary can be returned
const Vector2f* AP_Proximity_SITL::get_boundary_points(uint16_t& num_points) const
{
    // high-level status check
    if (state.status != AP_Proximity::Proximity_Good) {
        num_points = 0;
        return nullptr;
    }

    // return boundary points
    num_points = 8;
    return _boundary_point;
}

// update boundary points used for object avoidance based on a single sector's distance changing
//   the boundary points lie on the line between sectors meaning two boundary points may be updated based on a single sector's distance changing
//   the boundary point is set to the shortest distance found in the two adjacent sectors, this is a conservative boundary around the vehicle
void AP_Proximity_SITL::update_boundary_for_sector(uint8_t sector)
{
    // sanity check
    if (sector >= 8) {
        return;
    }

    // initialise sector_edge_vector if necessary
    if (_sector_edge_vector[sector].is_zero()) {
        float angle_rad = radians((45.0f * sector) + 22.5f);
        _sector_edge_vector[sector].x = cosf(angle_rad) * 100.0f;
        _sector_edge_vector[sector].y = sinf(angle_rad) * 100.0f;
    }

    // find adjacent sector (clockwise)
    uint8_t next_sector = sector + 1;
    if (next_sector >= 8) {
        next_sector = 0;
    }

    // boundary point lies on the line between the two sectors at the shorter distance found in the two sectors
    float shortest_distance = MIN(_distance[sector], _distance[next_sector]);
    _boundary_point[sector] = _sector_edge_vector[sector] * shortest_distance;

    // repeat for edge between sector and previous sector
    uint8_t prev_sector = (sector == 0) ? 8-1 : sector-1;
    shortest_distance = MIN(_distance[prev_sector], _distance[sector]);
    _boundary_point[prev_sector] = _sector_edge_vector[prev_sector] * shortest_distance;
}

#endif // CONFIG_HAL_BOARD
