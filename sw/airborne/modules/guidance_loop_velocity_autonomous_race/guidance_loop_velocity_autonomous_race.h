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
 * @file "modules/guidance_loop_velocity_autonomous_race/guidance_loop_velocity_autonomous_race.h"
 * @author Shuo Li
 * This module is used to control velocity only in MODULE mode
 */

#ifndef GUIDANCE_LOOP_VELOCITY_AUTONOMOUS_RACE_H
#define GUIDANCE_LOOP_VELOCITY_AUTONOMOUS_RACE_H

#include "math/pprz_algebra_int.h"

struct AutopilotMode
{
    uint8_t previousMode;
    uint8_t currentMode;
};

struct DESIRED_ATTITUDE_COMMAND
{

	float phi;
	float theta;
	float psi;
}; 


extern void guidance_h_module_init(void);
extern void guidance_h_module_run(bool in_flight);
extern void guidance_h_module_read_rc(void);
extern void guidance_h_module_enter(void);

extern struct AutopilotMode autopilotMode;
extern struct DESIRED_ATTITUDE_COMMAND attitude_cmd;
extern void guidance_h_module_get_mode_20HZ(void);

#endif



