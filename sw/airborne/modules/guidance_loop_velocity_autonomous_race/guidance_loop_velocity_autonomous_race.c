/*
 * Copyright (C) Shuo Li
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
 * @file "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.c"
 * @author Shuo Li
 * This module is used to control velocity only in MODULE mode
 */

#include "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
#include "state.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include<stdio.h>
#include<stdlib.h>
#include "autopilot.h"

struct AutopilotMode autopilotMode;
struct DESIRED_ATTITUDE_COMMAND attitude_cmd;
struct Int32Eulers attitude_cmd_i; 

void guidance_h_module_init(void) {
     autopilotMode.previousMode =  autopilot_get_mode();
     autopilotMode.currentMode = autopilot_get_mode();
}

void guidance_h_module_enter(void)
{
}

/**
 * Read the RC commands
 */
void guidance_h_module_read_rc(void)
{
}


void guidance_h_module_run(bool in_flight)    // this function is called in higher level in guidance_h.c
{
	printf("[guidance_loop_velocity] module mode is running\n");
     autopilotMode.currentMode = autopilot_get_mode();
     attitude_cmd_i.phi = BFP_OF_REAL(attitude_cmd.phi, INT32_ANGLE_FRAC);
     attitude_cmd_i.theta= BFP_OF_REAL(attitude_cmd.theta, INT32_ANGLE_FRAC);
     attitude_cmd_i.psi = BFP_OF_REAL(attitude_cmd.psi, INT32_ANGLE_FRAC);
     int32_quat_of_eulers(&stab_att_sp_quat,&attitude_cmd_i);
     stabilization_attitude_run(in_flight);
     autopilotMode.previousMode =  autopilotMode.currentMode;
}

