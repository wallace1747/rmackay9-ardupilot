#pragma once

#include <AP_Arming/AP_Arming.h>

/*
  a plane specific arming class
 */
class AP_Arming_Plane : public AP_Arming
{
public:
    AP_Arming_Plane(const AP_AHRS &ahrs_ref, const AP_Baro &baro, Compass &compass,
                    const AP_BattMonitor &battery) :
        AP_Arming(ahrs_ref, baro, compass, battery) {
            AP_Param::setup_object_defaults(this, var_info);
    }
    bool pre_arm_checks(bool report);
    bool arm(uint8_t method) override;

    // var_info for holding Parameter information
    static const struct AP_Param::GroupInfo var_info[];

protected:
    bool ins_checks(bool report);
    enum HomeState home_status() const override;

    // parameters
    AP_Int8                 rudder_arming_value;
};
