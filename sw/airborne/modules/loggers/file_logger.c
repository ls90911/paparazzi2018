/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** @file modules/loggers/file_logger.c
 *  @brief File logger for Linux based autopilots
 */

#include "file_logger.h"

#include <stdio.h>
#include "std.h"

#include "subsystems/imu.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "state.h"
#include "modules/sensors/mavlink_jevois.h"
#include "autopilot.h"
#include "modules/sonar/sonar_bebop.h"
#include "subsystems/ins/ins_int.h"

/** Set the default File logger path to the USB drive */
#ifndef FILE_LOGGER_PATH
#define FILE_LOGGER_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *file_logger = NULL;

/** Start the file logger and open a new file */
void file_logger_start(void)
{
  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  while ((file_logger = fopen(filename, "r"))) {
    fclose(file_logger);

    counter++;
    sprintf(filename, "%s/%05d.csv", STRINGIFY(FILE_LOGGER_PATH), counter);
  }

  file_logger = fopen(filename, "w");

  if (file_logger != NULL) {
    fprintf(
      file_logger,
      "counter,gyro_unscaled_p,gyro_unscaled_q,gyro_unscaled_r,accel_unscaled_x,accel_unscaled_y,accel_unscaled_z,mag_unscaled_x,mag_unscaled_y,mag_unscaled_z,COMMAND_THRUST,COMMAND_ROLL,COMMAND_PITCH,COMMAND_YAW,qi,qx,qy,qz\n"
    );
  }
}

/** Stop the logger an nicely close the file */
void file_logger_stop(void)
{
  if (file_logger != NULL) {
    fclose(file_logger);
    file_logger = NULL;
  }
}

/** Log the values to a csv file */
void file_logger_periodic(void)
{
  if (file_logger == NULL) {
    return;
  }
  static uint32_t counter;
  struct Int32Quat *quat = stateGetNedToBodyQuat_i();

  fprintf(file_logger, "%d,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%d,%f,%f,%f,%f,%f \n",
          counter,
          stateGetPositionNed_f()->x,
          stateGetPositionNed_f()->y,
          stateGetPositionNed_f()->z,
          stateGetSpeedNed_f()->x,
          stateGetSpeedNed_f()->y,
          stateGetSpeedNed_f()->z,
          stateGetNedToBodyEulers_f()->phi,
          stateGetNedToBodyEulers_f()->theta,
          stateGetNedToBodyEulers_f()->psi,
          stateGetBodyRates_f()->p,
          stateGetBodyRates_f()->q,
          stateGetBodyRates_f()->r,

	  kalmanFilterState.x,
	  kalmanFilterState.y,
	  kalmanFilterState.z,
          
	  imu.accel.x,
	  imu.accel.y,
	  imu.accel.z,
	  
	  get_butterworth_2_low_pass_int(&ax_filtered),
	  get_butterworth_2_low_pass_int(&ay_filtered),
	  get_butterworth_2_low_pass_int(&az_filtered),

	  attitude_cmd.phi,
	  attitude_cmd.theta,
	  attitude_cmd.psi,
	  attitude_cmd.alt,
		  
	  autopilot_get_mode(),

	 kalmanFilterState.bx, 
	 kalmanFilterState.by, 
	 kalmanFilterState.bz,

	 sonar_bebop.distance,

	 ins_int.baro_z


         );
  counter++;
}
