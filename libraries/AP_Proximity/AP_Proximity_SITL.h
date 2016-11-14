#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#include <AC_Fence/AC_PolyFence_loader.h>

class AP_Proximity_SITL : public AP_Proximity_Backend
{

public:
    // constructor
    AP_Proximity_SITL(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state);

    // get distance in meters in a particular direction in degrees (0 is forward, clockwise)
    // returns true on successful read and places distance in distance
    bool get_horizontal_distance(float angle_deg, float &distance) const override;

    // get boundary points around vehicle for use by avoidance
    //   returns nullptr and sets num_points to zero if no boundary can be returned
    const Vector2f* get_boundary_points(uint16_t& num_points) const override;

    // get distance and angle to closest object (used for pre-arm check)
    //   returns true on success, false if no valid readings
    bool get_closest_object(float& angle_deg, float &distance) const override;

    // update state
    void update(void) override;

private:
    SITL::SITL *sitl;
    Vector2l *fence;
    AP_Int8 *fence_count;
    uint32_t last_load_ms;
    AC_PolyFence_loader fence_loader;
    Location current_loc;

    // sector variables
    uint8_t last_sector;    // latest sector updated
    float _distance[8];     // distance in meters for each sector (0, 45, 90, 135, 180, 225, 270, 315)

    // fence boundary
    Vector2f _sector_edge_vector[8];    // vector for right-edge of each sector, used to speed up calculation of boundary
    Vector2f _boundary_point[8];        // bounding polygon around the vehicle calculated conservatively for object avoidance

    void load_fence(void);

    void update_boundary_for_sector(uint8_t sector);
};
#endif // CONFIG_HAL_BOARD
