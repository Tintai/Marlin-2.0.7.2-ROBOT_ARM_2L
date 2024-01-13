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

/**
 * llrobotarm.cpp
 */

#include "../inc/MarlinConfig.h"

#if IS_ROBOT_ARM_2L

#include "robotarm2l.h"
#include "motion.h"
#include "planner.h"
#include "endstops.h"

float delta_segments_per_second = ROBOT_ARM_2L_SEGMENTS_PER_SECOND;
float rot, high, low;

void ROBOT_ARM_2L_set_axis_is_at_home(const AxisEnum axis) {
    /*SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L ROBOT_ARM_2L_set_axis_is_at_home a=", axis
    );*/
    /**
     * ROBOT_ARM_2L homes XYZ at the same time
     */
    xyz_pos_t homeposition;
    LOOP_XYZ(i) homeposition[i] = base_home_pos((AxisEnum)i);

    
     /* SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L_set_axis_is_at_home homeposition.x=", homeposition.x,
      " homeposition.y=", homeposition.y,
      " homeposition.z=", homeposition.z
    );*/
    inverse_kinematics(homeposition);
    
    /*SERIAL_ECHOLNPAIR(
      "forward_kinematics_ROBOT_ARM_2L delta.a=", delta.a,
      " delta.b=", delta.b,
      " delta.c=", delta.c
    );*/
    forward_kinematics_ROBOT_ARM_2L(delta.a, delta.b, delta.c);
    current_position[axis] = cartes[axis];
    

    // SERIAL_ECHOPGM("Cartesian");
    // SERIAL_ECHOLNPAIR_P(SP_X_LBL, current_position.x, SP_Y_LBL, current_position.y);
    update_software_endstops(axis);
}

static constexpr xyz_pos_t ROBOT_ARM_2L_offset = { ROBOT_ARM_2L_OFFSET_X, ROBOT_ARM_2L_OFFSET_Y, ROBOT_ARM_2L_OFFSET_Z };

/**
 * ROBOT_ARM_2L Forward Kinematics. Results in 'cartes'.
 */
//2L TODO: split linkages to have different length, now assuming the same
void forward_kinematics_ROBOT_ARM_2L(const float &a, const float &b, const float &c) {
  
  const float rot_sin = sin(RADIANS(a)),
              rot_cos = cos(RADIANS(a)),
              low_sin = sin(RADIANS(b)),
              low_cos = cos(RADIANS(b)),
              PI_min_h_sin = sin(RADIANS(180 - c)),
              PI_min_h_cos = cos(RADIANS(180 - c));

  float rot_ee = ROBOT_ARM_2L_LOW_SHANK * low_sin + ROBOT_ARM_2L_HIGH_SHANK * PI_min_h_sin + ROBOT_ARM_2L_EE_OFFSET;

  float y = rot_ee * rot_sin;

  float x = rot_ee * rot_cos;

   /*SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L FK low_cos=", low_cos,
      " PI_min_h_cos=", PI_min_h_cos
    );*/

  float z = ROBOT_ARM_2L_LOW_SHANK * low_cos - ROBOT_ARM_2L_HIGH_SHANK * PI_min_h_cos;

  cartes.set(x + ROBOT_ARM_2L_offset.x, y + ROBOT_ARM_2L_offset.y, z + ROBOT_ARM_2L_offset.z);

  
    /*SERIAL_ECHOLNPAIR(
      "ROBOT_ARM_2L FK x=", x,
      " y=", y,
      " z=", z
    );*/
   // SERIAL_ECHOLNPAIR(" cartes (X,Y) = "(cartes.x, ", ", cartes.y, ")"));
  
}

void inverse_kinematics(const xyz_pos_t &raw) {
  
  const float L1sq = sq(ROBOT_ARM_2L_LOW_SHANK);
  const float L2sq = sq(ROBOT_ARM_2L_HIGH_SHANK);
  float rrot_ee = hypot(raw.x, raw.y);
  float rrot =  rrot_ee - ROBOT_ARM_2L_EE_OFFSET;
  float rside = hypot(rrot, raw.z);
  float RSsq = sq(rside);

  rot = acos(raw.x/ rrot_ee);

  high = PI - acos((L1sq + L2sq - RSsq) / (2 * ROBOT_ARM_2L_LOW_SHANK * ROBOT_ARM_2L_HIGH_SHANK));
  
  if (raw.z > 0) {
    low =  acos(raw.z / rside) - acos((L1sq - L2sq + RSsq) / (2 * ROBOT_ARM_2L_LOW_SHANK * rside));
  } else {
    low = PI - asin(rrot / rside) - acos((L1sq - L2sq + RSsq) / (2 * ROBOT_ARM_2L_LOW_SHANK * rside));
  }

  high = high + low;
  delta.set(DEGREES(rot), DEGREES(low), DEGREES(high));

}

void ROBOT_ARM_2L_report_positions() {
  SERIAL_ECHOLNPAIR("ROBOT_ARM_2L rot:", planner.get_axis_position_degrees(A_AXIS), "  low", planner.get_axis_position_degrees(B_AXIS), " high: ", planner.get_axis_position_degrees(C_AXIS));
  SERIAL_EOL();
}


void home_ROBOT_ARM_2L() {
    /*if(current_position.z < 0) {
      xyz_float_t safe_start_position = {current_position.x, current_position.y, 100};
      destination = safe_start_position;
      prepare_line_to_destination();
      planner.synchronize();
    }*/
    move_before_homing_ROBOT_ARM_2L();

    disable_all_steppers();

    homeaxis(Y_AXIS);
    homeaxis(Z_AXIS);  
    homeaxis(X_AXIS);  
    
    constexpr xyz_float_t endstop_backoff = {ROBOT_ARM_2L_X_AT_ENDSTOP, ROBOT_ARM_2L_Y_AT_ENDSTOP, ROBOT_ARM_2L_Z_AT_ENDSTOP};
    current_position = endstop_backoff;

    sync_plan_position();

    move_after_homing_ROBOT_ARM_2L();

    //TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(slow_homing));
}


void move_after_homing_ROBOT_ARM_2L() {
  current_position.set(MANUAL_X_HOME_POS, MANUAL_Y_HOME_POS, MANUAL_Z_HOME_POS);
  line_to_current_position(100);
  sync_plan_position();
}

void move_before_homing_ROBOT_ARM_2L() {
}

bool position_is_reachable_ROBOT_ARM_2L(const float &rx, const float &ry, const float &rz, const float inset) {
      //SERIAL_ECHOPAIR("position_is_reachable? rx: ", rx, ", ry: ", ry, ", rz:", rz, "\n");

      float rrot =  hypot(rx, ry) - ROBOT_ARM_2L_EE_OFFSET;    //radius from Top View
      float rrot_ee = hypot(rx, ry);
      float rrot_x = rrot * (rx / rrot_ee);
      float rrot_y = rrot * (ry / rrot_ee);
      float r2 = sq(rrot_x) + sq(rrot_y) + sq(rz);

      //2L TODO check inset
      bool retVal = (
          r2 <= sq(ROBOT_ARM_2L_MAX_RADIUS - inset) &&
          r2 >= sq(ROBOT_ARM_2L_MIN_RADIUS) && 
          rz >= Z_MIN_POS && 
          rz <= Z_MAX_POS &&  !(rx ==0 && ry==0)
      );
      //if(!retVal) {
      //  SERIAL_ECHOPAIR("r2:  ", r2, ", RMAX: ", ROBOT_ARM_2L_MAX_RADIUS, ", RMIN: ", ROBOT_ARM_2L_MIN_RADIUS, 
      //                ", ROBOT_ARM_2L_Z_MIN: ", Z_MIN_POS, ", ROBOT_ARM_2L_Z_MAX: ",Z_MAX_POS,"\n");
      //}
      //SERIAL_ECHOPAIR("position_is_reachable: ", retVal, "\n");
      return retVal;
}

#endif // IS_ROBOT_ARM_2L