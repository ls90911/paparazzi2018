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
 * @file "modules/sensors/mavlink_jevois.h"
 * @author MAVLab
 * Send sensor data to jevois and read commands from jevois
 */

#ifndef MAVLINK_JEVOIS_H
#define MAVLINK_JEVOIS_H




/*
 * Paparazzi UART over USB
 */
#include "mcu_periph/uart.h"
#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
#include "filters/low_pass_filter.h"

#ifndef JEVOIS_DEV
#define JEVOIS_DEV uart2
#endif

#define MAVLinkDev (&(JEVOIS_DEV).device)
#define MAVLinkTransmit(c) MAVLinkDev->put_byte(MAVLinkDev->periph, 0, c)
#define MAVLinkChAvailable() MAVLinkDev->char_available(MAVLinkDev->periph)
#define MAVLinkGetch() MAVLinkDev->get_byte(MAVLinkDev->periph)
#define MAVLinkSendMessage() MAVLinkDev->send_message(MAVLinkDev->periph, 0)





/*
 * MavLink protocol
 */

#include <mavlink/mavlink_types.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_ALIGNED_FIELDS 0

extern mavlink_system_t mavlink_system;

static inline void comm_send_ch(mavlink_channel_t chan __attribute__((unused)), uint8_t ch)
{
  // Send bytes
  MAVLinkTransmit(ch);
}



struct KALMAN_FILTER_STATE
{
	float x;
	float y;
	float z;
	float bx;
	float by;
	float bz;
};

extern struct KALMAN_FILTER_STATE kalmanFilterState;




/*
 * Paparazzi Module functions
 */


extern void mavlink_jevois_init(void);
extern void mavlink_jevois_periodic(void);
extern void mavlink_jevois_event(void);
extern void accelerator_filter_periodic(void);


extern Butterworth2LowPass_int ax_filtered;
extern Butterworth2LowPass_int ay_filtered;
extern Butterworth2LowPass_int az_filtered;


#endif

