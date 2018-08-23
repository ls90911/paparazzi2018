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
#include "subsystems/imu.h"
#include "autopilot.h"
#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"

mavlink_system_t mavlink_system;

#ifndef MAVLINK_SYSID
#define MAVLINK_SYSID 1
#endif





static void mavlink_send_heartbeat(void);
static void mavlink_send_attitude(void);
static void mavlink_send_highres_imu(void);
static void mavlink_send_set_mode(void);


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
  //RunOnceEvery(2, mavlink_send_attitude());
  RunOnceEvery(1, mavlink_send_highres_imu());
  RunOnceEvery(1, mavlink_send_set_mode());
}

void mavlink_jevois_event(void)
{
  mavlink_message_t msg;
  mavlink_status_t status;


  while (MAVLinkChAvailable()) {
    uint8_t c = MAVLinkGetch();
    if (mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {
	  printf("msg.msgid is %d\n",msg.msgid);
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

	case MAVLINK_MSG_ID_ATTITUDE:
	{

		// read attitude command from Jevois
		mavlink_attitude_t att;
		mavlink_msg_attitude_decode(&msg,&att);
		attitude_cmd.phi = att.roll;
		attitude_cmd.theta= att.pitch;
		attitude_cmd.psi = att.yaw;

		printf("[mavlink jevois] phi_cmd = %f\n",attitude_cmd.phi/3.14*180);
		printf("[mavlink jevois] theta_cmd = %f\n",attitude_cmd.theta/3.14*180);
		printf("[mavlink jevois] psi_cmd = %f\n",attitude_cmd.psi/3.14*180);
	}
	break;

	case MAVLINK_MSG_ID_ALTITUDE:
	{
		mavlink_altitude_t alt;
		mavlink_msg_altitude_decode(&msg,&alt);
		float altitude = alt.altitude_terrain;
		printf("[mavlink jevois] desired altitude is %f", altitude);
	}
	break;

	case MAVLINK_MSG_ID_DEBUG:
	{
		mavlink_debug_t debug;
		mavlink_msg_attitude_decode(&msg,&debug);
		printf("[mavlink jevois] debug value is %f", debug.value);
		printf("[mavlink jevois] debug ind is %d", debug.ind);
	}
	break;

	case MAVLINK_MSG_ID_MANUAL_SETPOINT:
	{
		mavlink_manual_setpoint_t cmd;
		mavlink_msg_manual_setpoint_decode(&msg,&cmd);
		attitude_cmd.phi = cmd.roll;
		attitude_cmd.theta= cmd.pitch;
		attitude_cmd.psi = cmd.yaw;
		attitude_cmd.alt= cmd.thrust;
		printf("[mavlink jevois] phi_cmd = %f\n",attitude_cmd.phi/3.14*180);
//		printf("[mavlink jevois] theta_cmd = %f\n",attitude_cmd.theta/3.14*180);
//		printf("[mavlink jevois] psi_cmd = %f\n",attitude_cmd.psi/3.14*180);
//		printf("[mavlink jevois] alt_cmd = %f\n",attitude_cmd.alt);
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


static void mavlink_send_set_mode(void)
{
//	printf("currentMode is %d\n",autopilotMode.currentMode);
	mavlink_msg_set_mode_send(MAVLINK_COMM_0,
                            get_sys_time_msec(),
			    autopilotMode.currentMode,
			   0 
			);
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


static void mavlink_send_highres_imu(void)
{
	mavlink_msg_highres_imu_send(MAVLINK_COMM_0,
			              get_sys_time_msec(),
				      imu.accel.x/1024.0,
				      imu.accel.y/1024.0,
				      imu.accel.z/1024.0,
				      stateGetBodyRates_f()->p,
				      stateGetBodyRates_f()->q,
				      stateGetBodyRates_f()->r,
				      stateGetNedToBodyEulers_f()->phi,
				      stateGetNedToBodyEulers_f()->theta,
				      stateGetNedToBodyEulers_f()->psi,
                                         0,
                                         0,
                                         0,
                                         0,
                                         0);
	MAVLinkSendMessage();
}



