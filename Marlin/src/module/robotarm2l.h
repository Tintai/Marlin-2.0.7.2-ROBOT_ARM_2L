/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * 2lrobotarm.h - ROBOT_ARM_2L-specific functions
 */

#include "../core/macros.h"

extern float delta_segments_per_second;


void ROBOT_ARM_2L_set_axis_is_at_home(const AxisEnum axis);
bool position_is_reachable_ROBOT_ARM_2L(const float &rx, const float &ry, const float &rz, const float inset);

void inverse_kinematics(const xyz_pos_t &raw);
void forward_kinematics_ROBOT_ARM_2L(const float &a, const float &b, const float &c);

void ROBOT_ARM_2L_report_positions();

void move_before_homing_ROBOT_ARM_2L();
void move_after_homing_ROBOT_ARM_2L();
void home_ROBOT_ARM_2L();