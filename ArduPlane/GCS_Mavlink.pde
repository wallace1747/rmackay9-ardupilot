// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Mavlink_compat.h"

// use this to prevent recursion during sensor init
static bool in_mavlink_delay;

// this costs us 51 bytes, but means that low priority
// messages don't block the CPU
static mavlink_statustext_t pending_status;


// check if a message will fit in the payload space available
#define CHECK_PAYLOAD_SIZE(id) if (payload_space < MAVLINK_MSG_ID_## id ##_LEN) return false

/*
  !!NOTE!!

  the use of NOINLINE separate functions for each message type avoids
  a compiler bug in gcc that would cause it to use far more stack
  space than is needed. Without the NOINLINE we use the sum of the
  stack needed for each message type. Please be careful to follow the
  pattern below when adding any new messages
 */

static NOINLINE void send_heartbeat(mavlink_channel_t chan)
{
#ifdef MAVLINK10
    uint8_t base_mode = 0;
    uint8_t system_status = MAV_STATE_ACTIVE;

    // we map the custom_mode to our internal mode plus 16, to lower
    // the chance that a ground station will give us 0 and we
    // interpret it as manual. This is necessary as the SET_MODE
    // command has no way to indicate that the custom_mode is filled in
    uint32_t custom_mode = control_mode + 16;

    // work out the base_mode. This value is almost completely useless
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (control_mode) {
    case MANUAL:
        base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;
    case STABILIZE:
    case FLY_BY_WIRE_A:
    case FLY_BY_WIRE_B:
    case FLY_BY_WIRE_C:
        base_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;
        break;
    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        base_mode = MAV_MODE_FLAG_GUIDED_ENABLED |
                    MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;
    case INITIALISING:
        system_status = MAV_STATE_CALIBRATING;
        break;
    }

    if (control_mode != MANUAL && control_mode != INITIALISING) {
        // stabiliser of some form is enabled
        base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
    }

#if ENABLE_STICK_MIXING==ENABLED
    if (control_mode != INITIALISING) {
        // all modes except INITIALISING have some form of manual
        // override if stick mixing is enabled
        base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    }
#endif

#if HIL_MODE != HIL_MODE_DISABLED
    base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
#endif

    // we are armed if we are not initialising
    if (control_mode != INITIALISING) {
        base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    mavlink_msg_heartbeat_send(
        chan,
        MAV_TYPE_FIXED_WING,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
#else // MAVLINK10
    mavlink_msg_heartbeat_send(
        chan,
        mavlink_system.type,
        MAV_AUTOPILOT_ARDUPILOTMEGA);
#endif // MAVLINK10
}

static NOINLINE void send_attitude(mavlink_channel_t chan)
{
    Vector3f omega = dcm.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        micros(),
        dcm.roll,
        dcm.pitch,
        dcm.yaw,
        omega.x,
        omega.y,
        omega.z);
}

static NOINLINE void send_extended_status1(mavlink_channel_t chan, uint16_t packet_drops)
{
#ifdef MAVLINK10
    uint32_t control_sensors_present = 0;
    uint32_t control_sensors_enabled;
    uint32_t control_sensors_health;

    // first what sensors/controllers we have
    control_sensors_present |= (1<<0); // 3D gyro present
    control_sensors_present |= (1<<1); // 3D accelerometer present
    if (g.compass_enabled) {
        control_sensors_present |= (1<<2); // compass present
    }
    control_sensors_present |= (1<<3); // absolute pressure sensor present
    if (g_gps->fix) {
        control_sensors_present |= (1<<5); // GPS present
    }
    control_sensors_present |= (1<<10); // 3D angular rate control
    control_sensors_present |= (1<<11); // attitude stabilisation
    control_sensors_present |= (1<<12); // yaw position
    control_sensors_present |= (1<<13); // altitude control
    control_sensors_present |= (1<<14); // X/Y position control
    control_sensors_present |= (1<<15); // motor control

    // now what sensors/controllers are enabled

    // first the sensors
    control_sensors_enabled = control_sensors_present & 0x1FF;

    // now the controllers
    control_sensors_enabled = control_sensors_present & 0x1FF;

    switch (control_mode) {
    case MANUAL:
        break;

    case STABILIZE:
    case FLY_BY_WIRE_A:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        break;

    case FLY_BY_WIRE_B:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case FLY_BY_WIRE_C:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<13); // altitude control
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case AUTO:
    case RTL:
    case LOITER:
    case GUIDED:
    case CIRCLE:
        control_sensors_enabled |= (1<<10); // 3D angular rate control
        control_sensors_enabled |= (1<<11); // attitude stabilisation
        control_sensors_enabled |= (1<<12); // yaw position
        control_sensors_enabled |= (1<<13); // altitude control
        control_sensors_enabled |= (1<<14); // X/Y position control
        control_sensors_enabled |= (1<<15); // motor control
        break;

    case INITIALISING:
        break;
    }

    // at the moment all sensors/controllers are assumed healthy
    control_sensors_health = control_sensors_present;

    uint16_t battery_current = -1;
    uint8_t  battery_remaining = -1;

    if (current_total != 0 && g.pack_capacity != 0) {
        battery_remaining = (100.0 * (g.pack_capacity - current_total) / g.pack_capacity);
    }
    if (current_total != 0) {
        battery_current = current_amps * 100;
    }

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        (uint16_t)(load * 1000),
        battery_voltage * 1000, // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0, // comm drops %,
        0, // comm drops in pkts,
        0, 0, 0, 0);

#else // MAVLINK10
        uint8_t mode 	 = MAV_MODE_UNINIT;
        uint8_t nav_mode = MAV_NAV_VECTOR;

        switch(control_mode) {
        case MANUAL:
            mode 		= MAV_MODE_MANUAL;
            break;
        case STABILIZE:
            mode 		= MAV_MODE_TEST1;
            break;
        case FLY_BY_WIRE_A:
            mode 		= MAV_MODE_TEST2;
            nav_mode 	= 1;				//FBW nav_mode mapping;  1=A, 2=B, 3=C, etc.
            break;
        case FLY_BY_WIRE_B:
            mode 		= MAV_MODE_TEST2;
            nav_mode 	= 2;				//FBW nav_mode mapping;  1=A, 2=B, 3=C, etc.
            break;
        case GUIDED:
            mode 		= MAV_MODE_GUIDED;
            break;
        case AUTO:
            mode 		= MAV_MODE_AUTO;
            nav_mode 	= MAV_NAV_WAYPOINT;
            break;
        case RTL:
            mode 		= MAV_MODE_AUTO;
            nav_mode 	= MAV_NAV_RETURNING;
            break;
        case LOITER:
            mode 		= MAV_MODE_AUTO;
            nav_mode 	= MAV_NAV_LOITER;
            break;
        case INITIALISING:
            mode 		= MAV_MODE_UNINIT;
            nav_mode 	= MAV_NAV_GROUNDED;
            break;
        }

        uint8_t status 		= MAV_STATE_ACTIVE;
        uint16_t battery_remaining = 1000.0 * (float)(g.pack_capacity - current_total)/(float)g.pack_capacity;	//Mavlink scaling 100% = 1000

        mavlink_msg_sys_status_send(
            chan,
            mode,
            nav_mode,
            status,
            load * 1000,
            battery_voltage * 1000,
            battery_remaining,
            packet_drops);
#endif // MAVLINK10
}

static void NOINLINE send_meminfo(mavlink_channel_t chan)
{
    extern unsigned __brkval;
    mavlink_msg_meminfo_send(chan, __brkval, memcheck_available_memory());
}

static void NOINLINE send_location(mavlink_channel_t chan)
{
    Matrix3f rot = dcm.get_dcm_matrix(); // neglecting angle of attack for now
#ifdef MAVLINK10
    mavlink_msg_global_position_int_send(
        chan,
        millis(),
        current_loc.lat,                // in 1E7 degrees
        current_loc.lng,                // in 1E7 degrees
        g_gps->altitude*10,             // millimeters above sea level
        current_loc.alt * 10,           // millimeters above ground
        g_gps->ground_speed * rot.a.x,  // X speed cm/s
        g_gps->ground_speed * rot.b.x,  // Y speed cm/s
        g_gps->ground_speed * rot.c.x,
        g_gps->ground_course);          // course in 1/100 degree
#else // MAVLINK10
    mavlink_msg_global_position_int_send(
        chan,
        current_loc.lat,
        current_loc.lng,
        current_loc.alt * 10,
        g_gps->ground_speed * rot.a.x,
        g_gps->ground_speed * rot.b.x,
        g_gps->ground_speed * rot.c.x);
#endif // MAVLINK10
}

static void NOINLINE send_nav_controller_output(mavlink_channel_t chan)
{
    mavlink_msg_nav_controller_output_send(
        chan,
        nav_roll / 1.0e2,
        nav_pitch / 1.0e2,
        nav_bearing / 1.0e2,
        target_bearing / 1.0e2,
        wp_distance,
        altitude_error / 1.0e2,
        airspeed_error,
        crosstrack_error);
}

static void NOINLINE send_gps_raw(mavlink_channel_t chan)
{
#ifdef MAVLINK10
    uint8_t fix;
    if (g_gps->status() == 2) {
        fix = 3;
    } else {
        fix = 0;
    }

    mavlink_msg_gps_raw_int_send(
        chan,
        micros(),
        fix,
        g_gps->latitude,      // in 1E7 degrees
        g_gps->longitude,     // in 1E7 degrees
        g_gps->altitude * 10, // in mm
        g_gps->hdop,
        65535,
        g_gps->ground_speed,  // cm/s
        g_gps->ground_course, // 1/100 degrees,
        g_gps->num_sats);

#else // MAVLINK10
        mavlink_msg_gps_raw_send(
            chan,
            micros(),
            g_gps->status(),
            g_gps->latitude / 1.0e7,
            g_gps->longitude / 1.0e7,
            g_gps->altitude / 100.0,
            g_gps->hdop,
            0.0,
            g_gps->ground_speed / 100.0,
            g_gps->ground_course / 100.0);
#endif  // MAVLINK10
}

static void NOINLINE send_servo_out(mavlink_channel_t chan)
{
    const uint8_t rssi = 1;
    // normalized values scaled to -10000 to 10000
    // This is used for HIL.  Do not change without discussing with
    // HIL maintainers
#ifdef MAVLINK10
    mavlink_msg_rc_channels_scaled_send(
        chan,
        millis(),
        0, // port 0
        10000 * g.channel_roll.norm_output(),
        10000 * g.channel_pitch.norm_output(),
        10000 * g.channel_throttle.norm_output(),
        10000 * g.channel_rudder.norm_output(),
        0,
        0,
        0,
        0,
        rssi);

#else // MAVLINK10
    mavlink_msg_rc_channels_scaled_send(
        chan,
        10000 * g.channel_roll.norm_output(),
        10000 * g.channel_pitch.norm_output(),
        10000 * g.channel_throttle.norm_output(),
        10000 * g.channel_rudder.norm_output(),
        0,
        0,
        0,
        0,
        rssi);
#endif // MAVLINK10
}

static void NOINLINE send_radio_in(mavlink_channel_t chan)
{
    uint8_t rssi = 1;
#ifdef MAVLINK10
    mavlink_msg_rc_channels_raw_send(
        chan,
        millis(),
        0, // port
        g.channel_roll.radio_in,
        g.channel_pitch.radio_in,
        g.channel_throttle.radio_in,
        g.channel_rudder.radio_in,
        g.rc_5.radio_in,       // XXX currently only 4 RC channels defined
        g.rc_6.radio_in,
        g.rc_7.radio_in,
        g.rc_8.radio_in,
        rssi);

#else // MAVLINK10
   mavlink_msg_rc_channels_raw_send(
        chan,
        g.channel_roll.radio_in,
        g.channel_pitch.radio_in,
        g.channel_throttle.radio_in,
        g.channel_rudder.radio_in,
        g.rc_5.radio_in,       // XXX currently only 4 RC channels defined
        g.rc_6.radio_in,
        g.rc_7.radio_in,
        g.rc_8.radio_in,
        rssi);
#endif // MAVLINK10
}

static void NOINLINE send_radio_out(mavlink_channel_t chan)
{
#ifdef MAVLINK10
        mavlink_msg_servo_output_raw_send(
            chan,
            micros(),
            0, // port
            g.channel_roll.radio_out,
            g.channel_pitch.radio_out,
            g.channel_throttle.radio_out,
            g.channel_rudder.radio_out,
            g.rc_5.radio_out,       // XXX currently only 4 RC channels defined
            g.rc_6.radio_out,
            g.rc_7.radio_out,
            g.rc_8.radio_out);
#else // MAVLINK10
        mavlink_msg_servo_output_raw_send(
            chan,
            g.channel_roll.radio_out,
            g.channel_pitch.radio_out,
            g.channel_throttle.radio_out,
            g.channel_rudder.radio_out,
            g.rc_5.radio_out,       // XXX currently only 4 RC channels defined
            g.rc_6.radio_out,
            g.rc_7.radio_out,
            g.rc_8.radio_out);
#endif // MAVLINK10
}

static void NOINLINE send_vfr_hud(mavlink_channel_t chan)
{
    mavlink_msg_vfr_hud_send(
        chan,
        (float)airspeed / 100.0,
        (float)g_gps->ground_speed / 100.0,
        (dcm.yaw_sensor / 100) % 360,
        (int)g.channel_throttle.servo_out,
        current_loc.alt / 100.0,
        0);
}

#if HIL_MODE != HIL_MODE_ATTITUDE
static void NOINLINE send_raw_imu1(mavlink_channel_t chan)
{
    Vector3f accel = imu.get_accel();
    Vector3f gyro = imu.get_gyro();

    mavlink_msg_raw_imu_send(
        chan,
        micros(),
        accel.x * 1000.0 / gravity,
        accel.y * 1000.0 / gravity,
        accel.z * 1000.0 / gravity,
        gyro.x * 1000.0,
        gyro.y * 1000.0,
        gyro.z * 1000.0,
        compass.mag_x,
        compass.mag_y,
        compass.mag_z);
}

static void NOINLINE send_raw_imu2(mavlink_channel_t chan)
{
    mavlink_msg_scaled_pressure_send(
        chan,
        micros(),
        (float)barometer.Press/100.0,
        (float)(barometer.Press-g.ground_pressure)/100.0,
        (int)(barometer.Temp*10));
}

static void NOINLINE send_raw_imu3(mavlink_channel_t chan)
{
    Vector3f mag_offsets = compass.get_offsets();

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.RawPress,
                                    barometer.RawTemp,
                                    imu.gx(), imu.gy(), imu.gz(),
                                    imu.ax(), imu.ay(), imu.az());
}
#endif // HIL_MODE != HIL_MODE_ATTITUDE

static void NOINLINE send_gps_status(mavlink_channel_t chan)
{
    mavlink_msg_gps_status_send(
        chan,
        g_gps->num_sats,
        NULL,
        NULL,
        NULL,
        NULL,
        NULL);
}

static void NOINLINE send_current_waypoint(mavlink_channel_t chan)
{
    mavlink_msg_waypoint_current_send(
        chan,
        g.waypoint_index);
}

static void NOINLINE send_statustext(mavlink_channel_t chan)
{
    mavlink_msg_statustext_send(
        chan,
        pending_status.severity,
        pending_status.text);
}


// try to send a message, return false if it won't fit in the serial tx buffer
static bool mavlink_try_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    int payload_space = comm_get_txspace(chan) - MAVLINK_NUM_NON_PAYLOAD_BYTES;

    if (chan == MAVLINK_COMM_1 && millis() < MAVLINK_TELEMETRY_PORT_DELAY) {
        // defer any messages on the telemetry port for 1 second after
        // bootup, to try to prevent bricking of Xbees
        return false;
    }

    switch (id) {
    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        send_heartbeat(chan);
        return true;

    case MSG_EXTENDED_STATUS1:
        CHECK_PAYLOAD_SIZE(SYS_STATUS);
        send_extended_status1(chan, packet_drops);
        break;

    case MSG_EXTENDED_STATUS2:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo(chan);
        break;

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude(chan);
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_location(chan);
        break;

    case MSG_NAV_CONTROLLER_OUTPUT:
        if (control_mode != MANUAL) {
            CHECK_PAYLOAD_SIZE(NAV_CONTROLLER_OUTPUT);
            send_nav_controller_output(chan);
        }
        break;

    case MSG_GPS_RAW:
#ifdef MAVLINK10
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
#else
        CHECK_PAYLOAD_SIZE(GPS_RAW);
#endif
        send_gps_raw(chan);
        break;

    case MSG_SERVO_OUT:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_SCALED);
        send_servo_out(chan);
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in(chan);
        break;

    case MSG_RADIO_OUT:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_radio_out(chan);
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud(chan);
        break;

#if HIL_MODE != HIL_MODE_ATTITUDE
    case MSG_RAW_IMU1:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu1(chan);
        break;

    case MSG_RAW_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_raw_imu2(chan);
        break;

    case MSG_RAW_IMU3:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_raw_imu3(chan);
        break;
#endif // HIL_MODE != HIL_MODE_ATTITUDE

    case MSG_GPS_STATUS:
        CHECK_PAYLOAD_SIZE(GPS_STATUS);
        send_gps_status(chan);
        break;

    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(WAYPOINT_CURRENT);
        send_current_waypoint(chan);
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_param_send();
        } else {
            gcs3.queued_param_send();
        }
        break;

    case MSG_NEXT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(WAYPOINT_REQUEST);
        if (chan == MAVLINK_COMM_0) {
            gcs0.queued_waypoint_send();
        } else {
            gcs3.queued_waypoint_send();
        }
        break;

    case MSG_STATUSTEXT:
        CHECK_PAYLOAD_SIZE(STATUSTEXT);
        send_statustext(chan);
        break;

    case MSG_RETRY_DEFERRED:
        break; // just here to prevent a warning
	}
    return true;
}


#define MAX_DEFERRED_MESSAGES MSG_RETRY_DEFERRED
static struct mavlink_queue {
    enum ap_message deferred_messages[MAX_DEFERRED_MESSAGES];
    uint8_t next_deferred_message;
    uint8_t num_deferred_messages;
} mavlink_queue[2];

// send a message using mavlink
static void mavlink_send_message(mavlink_channel_t chan, enum ap_message id, uint16_t packet_drops)
{
    uint8_t i, nextid;
    struct mavlink_queue *q = &mavlink_queue[(uint8_t)chan];

    // see if we can send the deferred messages, if any
    while (q->num_deferred_messages != 0) {
        if (!mavlink_try_send_message(chan,
                                      q->deferred_messages[q->next_deferred_message],
                                      packet_drops)) {
            break;
        }
        q->next_deferred_message++;
        if (q->next_deferred_message == MAX_DEFERRED_MESSAGES) {
            q->next_deferred_message = 0;
        }
        q->num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = q->next_deferred_message; i < q->num_deferred_messages; i++) {
        if (q->deferred_messages[nextid] == id) {
            // its already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MAX_DEFERRED_MESSAGES) {
            nextid = 0;
        }
    }

    if (q->num_deferred_messages != 0 ||
        !mavlink_try_send_message(chan, id, packet_drops)) {
        // can't send it now, so defer it
        if (q->num_deferred_messages == MAX_DEFERRED_MESSAGES) {
            // the defer buffer is full, discard
            return;
        }
        nextid = q->next_deferred_message + q->num_deferred_messages;
        if (nextid >= MAX_DEFERRED_MESSAGES) {
            nextid -= MAX_DEFERRED_MESSAGES;
        }
        q->deferred_messages[nextid] = id;
        q->num_deferred_messages++;
    }
}

void mavlink_send_text(mavlink_channel_t chan, gcs_severity severity, const char *str)
{
    if (chan == MAVLINK_COMM_1 && millis() < MAVLINK_TELEMETRY_PORT_DELAY) {
        // don't send status MAVLink messages for 2 seconds after
        // bootup, to try to prevent Xbee bricking
        return;
    }

    if (severity == SEVERITY_LOW) {
        // send via the deferred queuing system
        pending_status.severity = (uint8_t)severity;
        strncpy((char *)pending_status.text, str, sizeof(pending_status.text));
        mavlink_send_message(chan, MSG_STATUSTEXT, 0);
    } else {
        // send immediately
#ifdef MAVLINK10
        mavlink_msg_statustext_send(chan, severity, str);
#else
        mavlink_msg_statustext_send(chan, severity, (const int8_t*) str);
#endif
    }
}


GCS_MAVLINK::GCS_MAVLINK(AP_Var::Key key) :
packet_drops(0),

// parameters
// note, all values not explicitly initialised here are zeroed
waypoint_send_timeout(1000), // 1 second
waypoint_receive_timeout(1000), // 1 second

// stream rates
_group	(key, key == Parameters::k_param_streamrates_port0 ? PSTR("SR0_"): PSTR("SR3_")),
				// AP_VAR					//ref	 //index, default, 	name
				streamRateRawSensors		(&_group, 0, 		0,		 PSTR("RAW_SENS")),
				streamRateExtendedStatus	(&_group, 1, 		0,		 PSTR("EXT_STAT")),
				streamRateRCChannels		(&_group, 2, 		0,		 PSTR("RC_CHAN")),
				streamRateRawController		(&_group, 3, 		0,		 PSTR("RAW_CTRL")),
				streamRatePosition			(&_group, 4, 		0,		 PSTR("POSITION")),
				streamRateExtra1			(&_group, 5, 		0,		 PSTR("EXTRA1")),
				streamRateExtra2			(&_group, 6, 		0,		 PSTR("EXTRA2")),
				streamRateExtra3			(&_group, 7, 		0,		 PSTR("EXTRA3"))
{

}

void
GCS_MAVLINK::init(FastSerial * port)
{
    GCS_Class::init(port);
    if (port == &Serial) {
        mavlink_comm_0_port = port;
        chan = MAVLINK_COMM_0;
    }else{
        mavlink_comm_1_port = port;
        chan = MAVLINK_COMM_1;
    }
	_queued_parameter = NULL;
}

void
GCS_MAVLINK::update(void)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
	status.packet_rx_drop_count = 0;

    // process received bytes
    while (comm_get_available(chan))
    {
        uint8_t c = comm_receive_ch(chan);

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) handleMessage(&msg);
    }

    // Update packet drops counter
    packet_drops += status.packet_rx_drop_count;

    // send out queued params/ waypoints
    if (NULL != _queued_parameter) {
        send_message(MSG_NEXT_PARAM);
    }

    if (waypoint_receiving &&
        waypoint_request_i <= (unsigned)g.waypoint_total) {
        send_message(MSG_NEXT_WAYPOINT);
    }

    // stop waypoint sending if timeout
    if (waypoint_sending && (millis() - waypoint_timelast_send) > waypoint_send_timeout){
        waypoint_sending = false;
    }

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (millis() - waypoint_timelast_receive) > waypoint_receive_timeout){
        waypoint_receiving = false;
    }
}

void
GCS_MAVLINK::data_stream_send(uint16_t freqMin, uint16_t freqMax)
{
	if (waypoint_sending == false && waypoint_receiving == false && _queued_parameter == NULL) {

		if (freqLoopMatch(streamRateRawSensors, freqMin, freqMax)){
			send_message(MSG_RAW_IMU1);
			send_message(MSG_RAW_IMU2);
			send_message(MSG_RAW_IMU3);
		}

		if (freqLoopMatch(streamRateExtendedStatus, freqMin, freqMax)) {
			send_message(MSG_EXTENDED_STATUS1);
			send_message(MSG_EXTENDED_STATUS2);
			send_message(MSG_GPS_STATUS);
			send_message(MSG_CURRENT_WAYPOINT);
			send_message(MSG_GPS_RAW);            // TODO - remove this message after location message is working
			send_message(MSG_NAV_CONTROLLER_OUTPUT);
		}

		if (freqLoopMatch(streamRatePosition, freqMin, freqMax)) {
			// sent with GPS read
			send_message(MSG_LOCATION);
		}

		if (freqLoopMatch(streamRateRawController, freqMin, freqMax)) {
			// This is used for HIL.  Do not change without discussing with HIL maintainers
			send_message(MSG_SERVO_OUT);
		}

		if (freqLoopMatch(streamRateRCChannels, freqMin, freqMax)) {
			send_message(MSG_RADIO_OUT);
			send_message(MSG_RADIO_IN);
		}

		if (freqLoopMatch(streamRateExtra1, freqMin, freqMax)){	 // Use Extra 1 for AHRS info
			send_message(MSG_ATTITUDE);
		}

		if (freqLoopMatch(streamRateExtra2, freqMin, freqMax)){		// Use Extra 2 for additional HIL info
			send_message(MSG_VFR_HUD);
		}

		if (freqLoopMatch(streamRateExtra3, freqMin, freqMax)){
			// Available datastream
		}
	}
}



void
GCS_MAVLINK::send_message(enum ap_message id)
{
    mavlink_send_message(chan,id, packet_drops);
}

void
GCS_MAVLINK::send_text(gcs_severity severity, const char *str)
{
    mavlink_send_text(chan,severity,str);
}

void
GCS_MAVLINK::send_text(gcs_severity severity, const prog_char_t *str)
{
    mavlink_statustext_t m;
    uint8_t i;
    for (i=0; i<sizeof(m.text); i++) {
        m.text[i] = pgm_read_byte((const prog_char *)(str++));
    }
    if (i < sizeof(m.text)) m.text[i] = 0;
    mavlink_send_text(chan, severity, (const char *)m.text);
}

void GCS_MAVLINK::handleMessage(mavlink_message_t* msg)
{
    struct Location tell_command = {};                // command for telemetry
	static uint8_t mav_nav=255;							// For setting mode (some require receipt of 2 messages...)

    switch (msg->msgid) {

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        {
            // decode
            mavlink_request_data_stream_t packet;
            mavlink_msg_request_data_stream_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            int freq = 0; // packet frequency

			if (packet.start_stop == 0)
				freq = 0; // stop sending
			else if (packet.start_stop == 1)
				freq = packet.req_message_rate; // start sending
			else
				break;

            switch(packet.req_stream_id){

                case MAV_DATA_STREAM_ALL:
                    streamRateRawSensors = freq;
                    streamRateExtendedStatus = freq;
                    streamRateRCChannels = freq;
                    streamRateRawController = freq;
                    streamRatePosition = freq;
                    streamRateExtra1 = freq;
                    streamRateExtra2 = freq;
                    streamRateExtra3.set_and_save(freq);	// We just do set and save on the last as it takes care of the whole group.
                    break;

                case MAV_DATA_STREAM_RAW_SENSORS:
                    streamRateRawSensors = freq;		// We do not set and save this one so that if HIL is shut down incorrectly
														// we will not continue to broadcast raw sensor data at 50Hz.
                    break;
                case MAV_DATA_STREAM_EXTENDED_STATUS:
                    streamRateExtendedStatus.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_RC_CHANNELS:
                    streamRateRCChannels.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_RAW_CONTROLLER:
                    streamRateRawController.set_and_save(freq);
                    break;

				//case MAV_DATA_STREAM_RAW_SENSOR_FUSION:
				//    streamRateRawSensorFusion.set_and_save(freq);
				//    break;

                case MAV_DATA_STREAM_POSITION:
                    streamRatePosition.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA1:
                    streamRateExtra1.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA2:
                    streamRateExtra2.set_and_save(freq);
                    break;

                case MAV_DATA_STREAM_EXTRA3:
                    streamRateExtra3.set_and_save(freq);
                    break;

                default:
                    break;
            }
            break;
        }

#ifdef MAVLINK10
    case MAVLINK_MSG_ID_COMMAND_LONG:
        {
            // decode
            mavlink_command_long_t packet;
            mavlink_msg_command_long_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system, packet.target_component)) break;

            uint8_t result;

            // do command
            send_text(SEVERITY_LOW,PSTR("command received: "));

            switch(packet.command) {

            case MAV_CMD_NAV_LOITER_UNLIM:
                set_mode(LOITER);
                result = MAV_RESULT_ACCEPTED;
                break;

            case MAV_CMD_NAV_RETURN_TO_LAUNCH:
                set_mode(RTL);
                result = MAV_RESULT_ACCEPTED;
                break;

#if 0
                // not implemented yet, but could implement some of them
            case MAV_CMD_NAV_LAND:
            case MAV_CMD_NAV_TAKEOFF:
            case MAV_CMD_NAV_ROI:
            case MAV_CMD_NAV_PATHPLANNING:
                break;
#endif


            case MAV_CMD_PREFLIGHT_CALIBRATION:
                if (packet.param1 == 1 ||
                    packet.param2 == 1 ||
                    packet.param3 == 1) {
                    startup_IMU_ground();
                }
                if (packet.param4 == 1) {
                    trim_radio();
                }
                result = MAV_RESULT_ACCEPTED;
                break;


            default:
                result = MAV_RESULT_UNSUPPORTED;
                break;
            }

            mavlink_msg_command_ack_send(
                chan,
                packet.command,
                result);

            break;
        }

#else // MAVLINK10
    case MAVLINK_MSG_ID_ACTION:
        {
            // decode
            mavlink_action_t packet;
            mavlink_msg_action_decode(msg, &packet);
            if (mavlink_check_target(packet.target,packet.target_component)) break;

            uint8_t result = 0;

            // do action
            send_text(SEVERITY_LOW,PSTR("action received: "));
//Serial.println(packet.action);
            switch(packet.action){

                case MAV_ACTION_LAUNCH:
                    //set_mode(TAKEOFF);
                    break;

                case MAV_ACTION_RETURN:
                    set_mode(RTL);
                    result=1;
                    break;

                case MAV_ACTION_EMCY_LAND:
                    //set_mode(LAND);
                    break;

                case MAV_ACTION_HALT:
                    do_loiter_at_location();
                    result=1;
                    break;

                    /* No mappable implementation in APM 2.0
                case MAV_ACTION_MOTORS_START:
                case MAV_ACTION_CONFIRM_KILL:
                case MAV_ACTION_EMCY_KILL:
                case MAV_ACTION_MOTORS_STOP:
                case MAV_ACTION_SHUTDOWN:
                    break;
                */

                case MAV_ACTION_CONTINUE:
                    process_next_command();
                    result=1;
                    break;

                case MAV_ACTION_SET_MANUAL:
                    set_mode(MANUAL);
                    result=1;
                    break;

                case MAV_ACTION_SET_AUTO:
                    set_mode(AUTO);
                    result=1;
                    break;

                case MAV_ACTION_STORAGE_READ:
                    AP_Var::load_all();
                    result=1;
                    break;

                case MAV_ACTION_STORAGE_WRITE:
                    AP_Var::save_all();
                    result=1;
                    break;

                case MAV_ACTION_CALIBRATE_RC: break;
                    trim_radio();
                    result=1;
                    break;

                case MAV_ACTION_CALIBRATE_GYRO:
                case MAV_ACTION_CALIBRATE_MAG:
                case MAV_ACTION_CALIBRATE_ACC:
                case MAV_ACTION_CALIBRATE_PRESSURE:
                case MAV_ACTION_REBOOT:  // this is a rough interpretation
                    startup_IMU_ground();
                    result=1;
                    break;

                /*    For future implemtation
                case MAV_ACTION_REC_START: break;
                case MAV_ACTION_REC_PAUSE: break;
                case MAV_ACTION_REC_STOP: break;
                */

                /* Takeoff is not an implemented flight mode in APM 2.0
                case MAV_ACTION_TAKEOFF:
                    set_mode(TAKEOFF);
                    break;
                */

                case MAV_ACTION_NAVIGATE:
                    set_mode(AUTO);
                    result=1;
                    break;

                /* Land is not an implemented flight mode in APM 2.0
                case MAV_ACTION_LAND:
                    set_mode(LAND);
                    break;
                */

                case MAV_ACTION_LOITER:
                    set_mode(LOITER);
                    result=1;
                    break;

                default: break;
                }

                mavlink_msg_action_ack_send(
                    chan,
                    packet.action,
                    result
                    );

            break;
        }
#endif

    case MAVLINK_MSG_ID_SET_MODE:
		{
            // decode
            mavlink_set_mode_t packet;
            mavlink_msg_set_mode_decode(msg, &packet);

#ifdef MAVLINK10
            // we ignore base_mode as there is no sane way to map
            // from that bitmap to a APM flight mode. We rely on
            // custom_mode instead.
            // see comment on custom_mode above
            int16_t adjusted_mode = packet.custom_mode - 16;

            switch (adjusted_mode) {
            case MANUAL:
            case CIRCLE:
            case STABILIZE:
            case FLY_BY_WIRE_A:
            case FLY_BY_WIRE_B:
            case FLY_BY_WIRE_C:
            case AUTO:
            case RTL:
            case LOITER:
                set_mode(adjusted_mode);
                break;
            }

#else // MAVLINK10

            switch(packet.mode){

                case MAV_MODE_MANUAL:
					set_mode(MANUAL);
					break;

				case MAV_MODE_GUIDED:
					set_mode(GUIDED);
					break;

				case MAV_MODE_AUTO:
					if(mav_nav == 255 || mav_nav == MAV_NAV_WAYPOINT) 	set_mode(AUTO);
					if(mav_nav == MAV_NAV_RETURNING)					set_mode(RTL);
					if(mav_nav == MAV_NAV_LOITER)						set_mode(LOITER);
					mav_nav = 255;
					break;

                case MAV_MODE_TEST1:
					set_mode(STABILIZE);
					break;

                case MAV_MODE_TEST2:
					if(mav_nav == 255 || mav_nav == 1) 	set_mode(FLY_BY_WIRE_A);
					if(mav_nav == 2)					set_mode(FLY_BY_WIRE_B);
					//if(mav_nav == 3)					set_mode(FLY_BY_WIRE_C);
					mav_nav = 255;
					break;

			}
#endif
            break;
		}

#ifndef MAVLINK10
    case MAVLINK_MSG_ID_SET_NAV_MODE:
		{
            // decode
            mavlink_set_nav_mode_t packet;
            mavlink_msg_set_nav_mode_decode(msg, &packet);
			// To set some flight modes we must first receive a "set nav mode" message and then a "set mode" message
			mav_nav = packet.nav_mode;
			break;
		}
#endif // MAVLINK10


    case MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST:
        {
            // decode
            mavlink_waypoint_request_list_t packet;
            mavlink_msg_waypoint_request_list_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            // Start sending waypoints
            mavlink_msg_waypoint_count_send(
                chan,msg->sysid,
                msg->compid,
                g.waypoint_total + 1); // + home

            waypoint_timelast_send   = millis();
            waypoint_sending         = true;
            waypoint_receiving       = false;
            waypoint_dest_sysid      = msg->sysid;
            waypoint_dest_compid     = msg->compid;
            break;
        }


	// XXX read a WP from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_WAYPOINT_REQUEST:
        {
            // Check if sending waypiont
            //if (!waypoint_sending) break;
			// 5/10/11 - We are trying out relaxing the requirement that we be in waypoint sending mode to respond to a waypoint request.  DEW

            // decode
            mavlink_waypoint_request_t packet;
            mavlink_msg_waypoint_request_decode(msg, &packet);

 			if (mavlink_check_target(packet.target_system, packet.target_component))
 				break;

            // send waypoint
            tell_command = get_wp_with_index(packet.seq);

            // set frame of waypoint
            uint8_t frame;

			if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                frame = MAV_FRAME_GLOBAL_RELATIVE_ALT; // reference frame
            } else {
                frame = MAV_FRAME_GLOBAL; // reference frame
            }

            float param1 = 0, param2 = 0 , param3 = 0, param4 = 0;

            // time that the mav should loiter in milliseconds
            uint8_t current = 0; // 1 (true), 0 (false)

			if (packet.seq == (uint16_t)g.waypoint_index)
            	current = 1;

            uint8_t autocontinue = 1; // 1 (true), 0 (false)

            float x = 0, y = 0, z = 0;

            if (tell_command.id < MAV_CMD_NAV_LAST || tell_command.id == MAV_CMD_CONDITION_CHANGE_ALT) {
                // command needs scaling
                x = tell_command.lat/1.0e7; // local (x), global (latitude)
                y = tell_command.lng/1.0e7; // local (y), global (longitude)
                if (tell_command.options & MASK_OPTIONS_RELATIVE_ALT) {
                    z = (tell_command.alt - home.alt) / 1.0e2; // because tell_command.alt already includes a += home.alt
                } else {
                    z = tell_command.alt/1.0e2; // local (z), global/relative (altitude)
                }
            }

			switch (tell_command.id) {				// Switch to map APM command fields inot MAVLink command fields

				case MAV_CMD_NAV_LOITER_TURNS:
				case MAV_CMD_NAV_TAKEOFF:
				case MAV_CMD_DO_SET_HOME:
					param1 = tell_command.p1;
					break;

				case MAV_CMD_NAV_LOITER_TIME:
					param1 = tell_command.p1*10;    // APM loiter time is in ten second increments
					break;

				case MAV_CMD_CONDITION_CHANGE_ALT:
					x=0;	// Clear fields loaded above that we don't want sent for this command
					y=0;
				case MAV_CMD_CONDITION_DELAY:
				case MAV_CMD_CONDITION_DISTANCE:
					param1 = tell_command.lat;
					break;

				case MAV_CMD_DO_JUMP:
					param2 = tell_command.lat;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_REPEAT_SERVO:
					param4 = tell_command.lng;
				case MAV_CMD_DO_REPEAT_RELAY:
				case MAV_CMD_DO_CHANGE_SPEED:
					param3 = tell_command.lat;
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;

				case MAV_CMD_DO_SET_PARAMETER:
				case MAV_CMD_DO_SET_RELAY:
				case MAV_CMD_DO_SET_SERVO:
					param2 = tell_command.alt;
					param1 = tell_command.p1;
					break;
			}

			mavlink_msg_waypoint_send(chan,msg->sysid,
										msg->compid,
										packet.seq,
										frame,
										tell_command.id,
										current,
										autocontinue,
										param1,
										param2,
										param3,
										param4,
										x,
										y,
										z);

            // update last waypoint comm stamp
            waypoint_timelast_send = millis();
            break;
        }


    case MAVLINK_MSG_ID_WAYPOINT_ACK:
        {
            // decode
            mavlink_waypoint_ack_t packet;
            mavlink_msg_waypoint_ack_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // turn off waypoint send
            waypoint_sending = false;
            break;
        }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
        {
            // decode
            mavlink_param_request_list_t packet;
            mavlink_msg_param_request_list_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // Start sending parameters - next call to ::update will kick the first one out

            _queued_parameter = AP_Var::first();
            _queued_parameter_index = 0;
            _queued_parameter_count = _count_parameters();
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_CLEAR_ALL:
        {
            // decode
            mavlink_waypoint_clear_all_t packet;
            mavlink_msg_waypoint_clear_all_decode(msg, &packet);
			if (mavlink_check_target(packet.target_system, packet.target_component)) break;

            // clear all waypoints
            g.waypoint_total.set_and_save(0);

            // note that we don't send multiple acks, as otherwise a
            // GCS that is doing a clear followed by a set may see
            // the additional ACKs as ACKs of the set operations
            mavlink_msg_waypoint_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ACCEPTED);
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
        {
            // decode
            mavlink_waypoint_set_current_t packet;
            mavlink_msg_waypoint_set_current_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // set current command
            change_command(packet.seq);

            mavlink_msg_waypoint_current_send(chan, g.waypoint_index);
            break;
        }

    case MAVLINK_MSG_ID_WAYPOINT_COUNT:
        {
            // decode
            mavlink_waypoint_count_t packet;
            mavlink_msg_waypoint_count_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // start waypoint receiving
            if (packet.count > MAX_WAYPOINTS) {
                packet.count = MAX_WAYPOINTS;
            }
            g.waypoint_total.set_and_save(packet.count - 1);

            waypoint_timelast_receive = millis();
            waypoint_receiving   = true;
            waypoint_sending     = false;
            waypoint_request_i   = 0;
            break;
        }

#ifdef MAVLINK_MSG_ID_SET_MAG_OFFSETS
    case MAVLINK_MSG_ID_SET_MAG_OFFSETS:
    {
        mavlink_set_mag_offsets_t packet;
        mavlink_msg_set_mag_offsets_decode(msg, &packet);
        if (mavlink_check_target(packet.target_system,packet.target_component)) break;
        compass.set_offsets(Vector3f(packet.mag_ofs_x, packet.mag_ofs_y, packet.mag_ofs_z));
        break;
    }
#endif

	// XXX receive a WP from GCS and store in EEPROM
    case MAVLINK_MSG_ID_WAYPOINT:
        {
            // decode
            mavlink_waypoint_t packet;
            uint8_t result = MAV_MISSION_ACCEPTED;

            mavlink_msg_waypoint_decode(msg, &packet);
            if (mavlink_check_target(packet.target_system,packet.target_component)) break;

            // defaults
            tell_command.id = packet.command;

			switch (packet.frame)
			{
				case MAV_FRAME_MISSION:
				case MAV_FRAME_GLOBAL:
					{
						tell_command.lat = 1.0e7*packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7*packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z*1.0e2; // in as m converted to cm
						tell_command.options = 0; // absolute altitude
						break;
					}

#ifdef MAV_FRAME_LOCAL_NED
				case MAV_FRAME_LOCAL_NED: // local (relative to home position)
					{
						tell_command.lat = 1.0e7*ToDeg(packet.x/
						(radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
						tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = -packet.z*1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
#endif

#ifdef MAV_FRAME_LOCAL
				case MAV_FRAME_LOCAL: // local (relative to home position)
					{
						tell_command.lat = 1.0e7*ToDeg(packet.x/
						(radius_of_earth*cos(ToRad(home.lat/1.0e7)))) + home.lat;
						tell_command.lng = 1.0e7*ToDeg(packet.y/radius_of_earth) + home.lng;
						tell_command.alt = packet.z*1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT;
						break;
					}
#endif

				case MAV_FRAME_GLOBAL_RELATIVE_ALT: // absolute lat/lng, relative altitude
					{
						tell_command.lat = 1.0e7 * packet.x; // in as DD converted to * t7
						tell_command.lng = 1.0e7 * packet.y; // in as DD converted to * t7
						tell_command.alt = packet.z * 1.0e2;
						tell_command.options = MASK_OPTIONS_RELATIVE_ALT; // store altitude relative!! Always!!
						break;
					}

            default:
                result = MAV_MISSION_UNSUPPORTED_FRAME;
                break;
			}

            
            if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

            switch (tell_command.id) {                    // Switch to map APM command fields inot MAVLink command fields
            case MAV_CMD_NAV_LOITER_TURNS:
            case MAV_CMD_NAV_TAKEOFF:
            case MAV_CMD_DO_SET_HOME:
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_CONDITION_CHANGE_ALT:
                tell_command.lat = packet.param1;
                break;

            case MAV_CMD_NAV_LOITER_TIME:
                tell_command.p1 = packet.param1 / 10;    // APM loiter time is in ten second increments
                break;

            case MAV_CMD_CONDITION_DELAY:
            case MAV_CMD_CONDITION_DISTANCE:
                tell_command.lat = packet.param1;
                break;

            case MAV_CMD_DO_JUMP:
                tell_command.lat = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_DO_REPEAT_SERVO:
                tell_command.lng = packet.param4;
            case MAV_CMD_DO_REPEAT_RELAY:
            case MAV_CMD_DO_CHANGE_SPEED:
                tell_command.lat = packet.param3;
                tell_command.alt = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            case MAV_CMD_DO_SET_PARAMETER:
            case MAV_CMD_DO_SET_RELAY:
            case MAV_CMD_DO_SET_SERVO:
                tell_command.alt = packet.param2;
                tell_command.p1 = packet.param1;
                break;

            default:
                result = MAV_MISSION_UNSUPPORTED;
                break;
            }

            if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

			if(packet.current == 2){ 				//current = 2 is a flag to tell us this is a "guided mode" waypoint and not for the mission
				guided_WP = tell_command;

				// add home alt if needed
				if (guided_WP.options & MASK_OPTIONS_RELATIVE_ALT){
					guided_WP.alt += home.alt;
				}

				set_mode(GUIDED);

				// make any new wp uploaded instant (in case we are already in Guided mode)
				set_guided_WP();

				// verify we recevied the command
				mavlink_msg_waypoint_ack_send(
						chan,
						msg->sysid,
						msg->compid,
						0);

			} else {
				// Check if receiving waypoints (mission upload expected)
				if (!waypoint_receiving) {
                    result = MAV_MISSION_ERROR;
                    goto mission_failed;
                }

				// check if this is the requested waypoint
				if (packet.seq != waypoint_request_i) {
                    result = MAV_MISSION_INVALID_SEQUENCE;
                    goto mission_failed;
                }

                set_wp_with_index(tell_command, packet.seq);

				// update waypoint receiving state machine
				waypoint_timelast_receive = millis();
				waypoint_request_i++;

				if (waypoint_request_i > (uint16_t)g.waypoint_total){
					mavlink_msg_waypoint_ack_send(
						chan,
						msg->sysid,
						msg->compid,
						result);

					send_text(SEVERITY_LOW,PSTR("flight plan received"));
					waypoint_receiving = false;
					// XXX ignores waypoint radius for individual waypoints, can
					// only set WP_RADIUS parameter
				}
			}
            break;

        mission_failed:
            // we are rejecting the mission/waypoint
            mavlink_msg_waypoint_ack_send(
                chan,
                msg->sysid,
                msg->compid,
                result);
            break;
        }

    case MAVLINK_MSG_ID_PARAM_SET:
        {
            AP_Var                  *vp;
            AP_Meta_class::Type_id  var_type;

            // decode
            mavlink_param_set_t packet;
            mavlink_msg_param_set_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system, packet.target_component))
				break;

            // set parameter

            char key[ONBOARD_PARAM_NAME_LENGTH+1];
            strncpy(key, (char *)packet.param_id, ONBOARD_PARAM_NAME_LENGTH);
            key[ONBOARD_PARAM_NAME_LENGTH] = 0;

            // find the requested parameter
            vp = AP_Var::find(key);
            if ((NULL != vp) &&                             // exists
                    !isnan(packet.param_value) &&               // not nan
                    !isinf(packet.param_value)) {               // not inf

                // add a small amount before casting parameter values
                // from float to integer to avoid truncating to the
                // next lower integer value.
				float rounding_addition = 0.01;

                // fetch the variable type ID
                var_type = vp->meta_type_id();

                // handle variables with standard type IDs
                if (var_type == AP_Var::k_typeid_float) {
                    ((AP_Float *)vp)->set_and_save(packet.param_value);
                } else if (var_type == AP_Var::k_typeid_float16) {
                    ((AP_Float16 *)vp)->set_and_save(packet.param_value);
                } else if (var_type == AP_Var::k_typeid_int32) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    ((AP_Int32 *)vp)->set_and_save(packet.param_value+rounding_addition);
                } else if (var_type == AP_Var::k_typeid_int16) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    ((AP_Int16 *)vp)->set_and_save(packet.param_value+rounding_addition);
                } else if (var_type == AP_Var::k_typeid_int8) {
                    if (packet.param_value < 0) rounding_addition = -rounding_addition;
                    ((AP_Int8 *)vp)->set_and_save(packet.param_value+rounding_addition);
                } else {
                    // we don't support mavlink set on this parameter
                    break;
                }

                // Report back the new value if we accepted the change
                // we send the value we actually set, which could be
                // different from the value sent, in case someone sent
                // a fractional value to an integer type
#ifdef MAVLINK10
                mavlink_msg_param_value_send(
                    chan,
                    key,
                    vp->cast_to_float(),
                    mav_var_type(vp->meta_type_id()),
                    _count_parameters(),
                    -1); // XXX we don't actually know what its index is...
#else // MAVLINK10
                mavlink_msg_param_value_send(
                    chan,
                    (int8_t *)key,
                    vp->cast_to_float(),
                    _count_parameters(),
                    -1); // XXX we don't actually know what its index is... 
#endif // MAVLINK10
            }

            break;
        } // end case

    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
        {
            // allow override of RC channel values for HIL
            // or for complete GCS control of switch position
            // and RC PWM values.
			if(msg->sysid != g.sysid_my_gcs) break;		// Only accept control from our gcs
            mavlink_rc_channels_override_t packet;
            int16_t v[8];
            mavlink_msg_rc_channels_override_decode(msg, &packet);

			if (mavlink_check_target(packet.target_system,packet.target_component))
				break;

            v[0] = packet.chan1_raw;
            v[1] = packet.chan2_raw;
            v[2] = packet.chan3_raw;
            v[3] = packet.chan4_raw;
            v[4] = packet.chan5_raw;
            v[5] = packet.chan6_raw;
            v[6] = packet.chan7_raw;
            v[7] = packet.chan8_raw;
            rc_override_active = APM_RC.setHIL(v);
			rc_override_fs_timer = millis();
            break;
        }

    case MAVLINK_MSG_ID_HEARTBEAT:
        {
            // We keep track of the last time we received a heartbeat from our GCS for failsafe purposes
			if(msg->sysid != g.sysid_my_gcs) break;
			rc_override_fs_timer = millis();
			pmTest1++;
            break;
        }

	#if HIL_MODE != HIL_MODE_DISABLED
        // This is used both as a sensor and to pass the location
        // in HIL_ATTITUDE mode.
#ifdef MAVLINK10
	case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
            // decode
            mavlink_gps_raw_int_t packet;
            mavlink_msg_gps_raw_int_decode(msg, &packet);

            // set gps hil sensor
            g_gps->setHIL(packet.time_usec/1000.0,
                          packet.lat*1.0e-7, packet.lon*1.0e-7, packet.alt*1.0e-3,
                          packet.vel*1.0e-2, packet.cog*1.0e-2, 0, 0);
            break;
        }
#else // MAVLINK10
	case MAVLINK_MSG_ID_GPS_RAW:
        {
            // decode
            mavlink_gps_raw_t packet;
            mavlink_msg_gps_raw_decode(msg, &packet);

            // set gps hil sensor
            g_gps->setHIL(packet.usec/1000.0,packet.lat,packet.lon,packet.alt,
            packet.v,packet.hdg,0,0);
            break;
        }
#endif // MAVLINK10

        //    Is this resolved? - MAVLink protocol change.....
    case MAVLINK_MSG_ID_VFR_HUD:
        {
            // decode
            mavlink_vfr_hud_t packet;
            mavlink_msg_vfr_hud_decode(msg, &packet);

            // set airspeed
            airspeed = 100*packet.airspeed;
            break;
        }

#endif
#if HIL_MODE == HIL_MODE_ATTITUDE
    case MAVLINK_MSG_ID_ATTITUDE:
        {
            // decode
            mavlink_attitude_t packet;
            mavlink_msg_attitude_decode(msg, &packet);

            // set dcm hil sensor
            dcm.setHil(packet.roll,packet.pitch,packet.yaw,packet.rollspeed,
            packet.pitchspeed,packet.yawspeed);
            break;
        }
#endif
#if HIL_MODE == HIL_MODE_SENSORS

    case MAVLINK_MSG_ID_RAW_IMU:
        {
            // decode
            mavlink_raw_imu_t packet;
            mavlink_msg_raw_imu_decode(msg, &packet);

            // set imu hil sensors
            // TODO: check scaling for temp/absPress
            float temp = 70;
            float absPress = 1;
                  //Serial.printf_P(PSTR("accel: %d %d %d\n"), packet.xacc, packet.yacc, packet.zacc);
                  //Serial.printf_P(PSTR("gyro: %d %d %d\n"), packet.xgyro, packet.ygyro, packet.zgyro);

            // rad/sec
            Vector3f gyros;
            gyros.x = (float)packet.xgyro / 1000.0;
            gyros.y = (float)packet.ygyro / 1000.0;
            gyros.z = (float)packet.zgyro / 1000.0;
            // m/s/s
            Vector3f accels;
            accels.x = (float)packet.xacc / 1000.0;
            accels.y = (float)packet.yacc / 1000.0;
            accels.z = (float)packet.zacc / 1000.0;

            imu.set_gyro(gyros);

            imu.set_accel(accels);

            compass.setHIL(packet.xmag,packet.ymag,packet.zmag);
            break;
        }

    case MAVLINK_MSG_ID_RAW_PRESSURE:
        {
            // decode
            mavlink_raw_pressure_t packet;
            mavlink_msg_raw_pressure_decode(msg, &packet);

            // set pressure hil sensor
            // TODO: check scaling
            float temp = 70;
            barometer.setHIL(temp,packet.press_diff1 + 101325);
            break;
        }
#endif // HIL_MODE
    } // end switch
} // end handle mavlink

uint16_t
GCS_MAVLINK::_count_parameters()
{
    // if we haven't cached the parameter count yet...
    if (0 == _parameter_count) {
        AP_Var  *vp;

        vp = AP_Var::first();
        do {
            // if a parameter responds to cast_to_float then we are going to be able to report it
            if (!isnan(vp->cast_to_float())) {
                _parameter_count++;
            }
        } while (NULL != (vp = vp->next()));
    }
    return _parameter_count;
}

AP_Var *
GCS_MAVLINK::_find_parameter(uint16_t index)
{
    AP_Var  *vp;

    vp = AP_Var::first();
    while (NULL != vp) {

        // if the parameter is reportable
        if (!(isnan(vp->cast_to_float()))) {
            // if we have counted down to the index we want
            if (0 == index) {
                // return the parameter
                return vp;
            }
            // count off this parameter, as it is reportable but not
            // the one we want
            index--;
        }
        // and move to the next parameter
        vp = vp->next();
    }
    return NULL;
}

/**
* @brief Send the next pending parameter, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_param_send()
{
    // Check to see if we are sending parameters
    if (NULL == _queued_parameter) return;

    AP_Var      *vp;
    float       value;

    // copy the current parameter and prepare to move to the next
    vp = _queued_parameter;
    _queued_parameter = _queued_parameter->next();

    // if the parameter can be cast to float, report it here and break out of the loop
    value = vp->cast_to_float();
    if (!isnan(value)) {
        char param_name[ONBOARD_PARAM_NAME_LENGTH];         /// XXX HACK
        vp->copy_name(param_name, sizeof(param_name));

#ifdef MAVLINK10
        mavlink_msg_param_value_send(
            chan,
            param_name,
            value,
            mav_var_type(vp->meta_type_id()),
            _queued_parameter_count,
            _queued_parameter_index);
#else // MAVLINK10
        mavlink_msg_param_value_send(
            chan,
            (int8_t*)param_name,
            value,
            _queued_parameter_count,
            _queued_parameter_index);
#endif // MAVLINK10

        _queued_parameter_index++;
    }
}

/**
* @brief Send the next pending waypoint, called from deferred message
* handling code
*/
void
GCS_MAVLINK::queued_waypoint_send()
{
    if (waypoint_receiving &&
        waypoint_request_i <= (unsigned)g.waypoint_total) {
        mavlink_msg_waypoint_request_send(
            chan,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i);
    }
}

/*
 a delay() callback that processes MAVLink packets. We set this as the
 callback in long running library initialisation routines to allow
 MAVLink to process packets while waiting for the initialisation to
 complete
*/
static void mavlink_delay(unsigned long t)
{
    unsigned long tstart;
    static unsigned long last_1hz, last_50hz;

    if (in_mavlink_delay) {
        // this should never happen, but let's not tempt fate by
        // letting the stack grow too much
        delay(t);
        return;
    }

    in_mavlink_delay = true;

    tstart = millis();
    do {
        unsigned long tnow = millis();
        if (tnow - last_1hz > 1000) {
            last_1hz = tnow;
            gcs_send_message(MSG_HEARTBEAT);
            gcs_send_message(MSG_EXTENDED_STATUS1);
        }
        if (tnow - last_50hz > 20) {
            last_50hz = tnow;
            gcs_update();
        }
        delay(1);
    } while (millis() - tstart < t);

    in_mavlink_delay = false;
}

/*
  send a message on both GCS links
 */
static void gcs_send_message(enum ap_message id)
{
    gcs0.send_message(id);
    gcs3.send_message(id);
}

/*
  send data streams in the given rate range on both links
 */
static void gcs_data_stream_send(uint16_t freqMin, uint16_t freqMax)
{
    gcs0.data_stream_send(freqMin, freqMax);
    gcs3.data_stream_send(freqMin, freqMax);
}

/*
  look for incoming commands on the GCS links
 */
static void gcs_update(void)
{
	gcs0.update();
    gcs3.update();
}

static void gcs_send_text(gcs_severity severity, const char *str)
{
    gcs0.send_text(severity, str);
    gcs3.send_text(severity, str);
}

static void gcs_send_text_P(gcs_severity severity, const prog_char_t *str)
{
    gcs0.send_text(severity, str);
    gcs3.send_text(severity, str);
}

/*
  send a low priority formatted message to the GCS
  only one fits in the queue, so if you send more than one before the
  last one gets into the serial buffer then the old one will be lost
 */
static void gcs_send_text_fmt(const prog_char_t *fmt, ...)
{
    char fmtstr[40];
    va_list ap;
    uint8_t i;
    for (i=0; i<sizeof(fmtstr)-1; i++) {
        fmtstr[i] = pgm_read_byte((const prog_char *)(fmt++));
        if (fmtstr[i] == 0) break;
    }
    fmtstr[i] = 0;
    pending_status.severity = (uint8_t)SEVERITY_LOW;
    va_start(ap, fmt);
    vsnprintf((char *)pending_status.text, sizeof(pending_status.text), fmtstr, ap);
    va_end(ap);
    mavlink_send_message(MAVLINK_COMM_0, MSG_STATUSTEXT, 0);
    mavlink_send_message(MAVLINK_COMM_1, MSG_STATUSTEXT, 0);
}
