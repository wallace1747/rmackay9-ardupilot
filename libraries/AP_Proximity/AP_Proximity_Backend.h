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
#include <AP_HAL/AP_HAL.h>
#include "AP_Proximity.h"

class AP_Proximity_Backend
{
public:
    // constructor. This incorporates initialisation as well.
	AP_Proximity_Backend(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // we declare a virtual destructor so that Proximity drivers can
    // override with a custom destructor if need be
    virtual ~AP_Proximity_Backend(void) {}

    // update the state structure
    virtual void update() = 0;

    // get distance in meters in a particular direction in degrees (0 is forward, clockwise)
    // returns true on successful read and places distance in distance
    virtual bool get_horizontal_distance(float angle_deg, float &distance) const = 0;

    // get boundary points around vehicle for use by avoidance
    //   returns nullptr and sets num_points to zero if no boundary can be returned
    virtual const Vector2f* get_boundary_points(uint16_t& num_points) const = 0;

    // get distance and angle to closest object (used for pre-arm check)
    //   returns true on success, false if no valid readings
    virtual bool get_closest_object(float& angle_deg, float &distance) const = 0;

protected:

    // set status and update valid_count
    void set_status(AP_Proximity::Proximity_Status status);

    // get ignore area info
    uint8_t get_ignore_area_count() const;
    bool get_ignore_area(uint8_t index, uint16_t &angle_deg, uint8_t &width_deg) const;
    bool get_next_ignore_start_or_end(uint8_t start_or_end, int16_t start_angle, int16_t &ignore_start) const;

    AP_Proximity &frontend;
    AP_Proximity::Proximity_State &state;   // reference to this instances state
};
