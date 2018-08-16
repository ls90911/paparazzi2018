/*
 * Copyright (C) MAVLab
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/sensors/mavlink_jevois.c"
 * @author MAVLab
 * Send sensor data to jevois and read commands from jevois
 */

#include "modules/sensors/mavlink_jevois.h"



/*
 * MavLink protocol
 */

#include <mavlink/mavlink_types.h>
#include "mavlink/paparazzi/mavlink.h"

mavlink_system_t mavlink_system;

#ifndef MAVLINK_SYSID
#define MAVLINK_SYSID 1
#endif





static void mavlink_send_heartbeat(void);
static void mavlink_send_attitude(void);


/*
 * Paparazzi Module functions
 */


void mavlink_jevois_init(void)
{
  mavlink_system.sysid = MAVLINK_SYSID; // System ID, 1-255
  mavlink_system.compid = 0; // Component/Subsystem ID, 1-255
}

void mavlink_jevois_periodic(void)
{
  RunOnceEvery(100, mavlink_send_heartbeat());
  RunOnceEvery(2, mavlink_send_attitude());
}

void mavlink_jevois_event(void)
{
  mavlink_message_t msg;
  mavlink_status_t status;


  while (MAVLinkChAvailable()) {
    uint8_t c = MAVLinkGetch();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      switch (msg.msgid) {

        case MAVLINK_MSG_ID_HEARTBEAT: {
          mavlink_heartbeat_t heartbeat;
          mavlink_msg_heartbeat_decode(&msg, &heartbeat);
          // do something with heartbeat variable
        }
        break;

        case MAVLINK_MSG_ID_HIGHRES_IMU: {
          mavlink_highres_imu_t imu;
          mavlink_msg_highres_imu_decode(&msg, &imu);
          // use
          int i = imu.xacc;
        }
        break;
      }
    }
  }
}


/////////////////////////////


#include "state.h"
#include "mcu_periph/sys_time.h"



static void mavlink_send_attitude(void)
{
  mavlink_msg_attitude_send(MAVLINK_COMM_0,
                            get_sys_time_msec(),
                            stateGetNedToBodyEulers_f()->phi,     // Phi
                            stateGetNedToBodyEulers_f()->theta,   // Theta
                            stateGetNedToBodyEulers_f()->psi,     // Psi
                            stateGetBodyRates_f()->p,             // p
                            stateGetBodyRates_f()->q,             // q
                            stateGetBodyRates_f()->r);            // r
  MAVLinkSendMessage();
}


static void mavlink_send_heartbeat(void)
{
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0,
                             MAV_TYPE_FIXED_WING,
                             MAV_AUTOPILOT_PPZ,
                             MAV_MODE_FLAG_AUTO_ENABLED,
                             0, // custom_mode
                             MAV_STATE_CALIBRATING);
  MAVLinkSendMessage();
}





