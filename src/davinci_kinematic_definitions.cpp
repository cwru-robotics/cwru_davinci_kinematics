/*
 *  davinci_kinematic_definitions.h
 *  Copyright (C) 2017  Wyatt S. Newman, Russell C. Jackson, and Tom Shkurti.

 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.

 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.

 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cwru_davinci_kinematics/davinci_kinematic_definitions.h>
#include <math.h>

namespace davinci_kinematics
{

// origin0 coincident w/ origin1
 const double DH_a1 = 0.0;
// const double DH_a1 = 0.0059; //RN 20180218A1
// const double DH_a1 = 0.001; //RN 20180218A2
// const double DH_a1 = 0.001420; //RN 20180218A3
// const double DH_a1 = 0.0005; //RN 20180219A1
// const double DH_a1 = 0.0045; //RN 20180219A2
// const double DH_a1 = -0.001; //RN 20180219A3
// const double DH_a1 = 0.000; //RN 20180220A1	
// const double DH_a1 = 0.0086; //RN 20180220A2	
// const double DH_a1 = -0.001; //RN 20180222A3
// const double DH_a1 = 0.0059; //RN 20180319A1

// axis z1,z2 (prismatic) intersect
 const double DH_a2 = 0.00;
// const double DH_a2 = 0.002; //RN 20180215A1
// const double DH_a2 = 0.0005; //RN 20180216
// const double DH_a2 = 0.0012; //RN 20180218
// const double DH_a2 = 0.0012; //RN 20180218A2
// const double DH_a2 = 0.001340; //RN 20180219A1 
// const double DH_a2 = 0.0013; // RN 20180222 GOOD
// const double DH_a2 = 0.0012; // RN 20180319A1 BAD
// const double DH_a2 = 0.0010; // RN 20180319A2

// const double DH_a2 = 0.0000; // RN 20180319PSM2A1


// axes z2 (prismatic) and z3 (shaft rot) intersect
const double DH_a3 = 0.0;
// axes z3 (shaft rot) and z4 (wrist bend) intersect
const double DH_a4 = 0.0;
// axes z4 (wrist bend) and z5 (gripper-jaw rot axis) do NOT intersect:
const double DH_a5 = dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis;
// define tool frame on jaw w/ z6 axis intersecting z5 axis
const double DH_a6 = 0.0;
// not sure what to do with this one
const double DH_a7 = 0.0;

// const double DH_d1 = 0.0;
// //  THIS IS VARIABLE
// const double DH_d2 = 0.0;
// const double DH_d3 = 0.0;
// const double DH_d4 = 0.0;
// const double DH_d5 = 0.0;
// const double DH_d6 = 0.0;
// const double DH_d7 = 0.0;

// robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';


const double DH_alpha1 = M_PI/2.0;
// const double DH_alpha1 =  1.5774; // RN 20180218A1
// const double DH_alpha1 =  1.5644206; // RN 20180218A2 << GOOD

// const double DH_alpha1 =  1.5765; // RN 20180319PSM2A1

const double DH_alpha2 = M_PI/2.0;
// const double DH_alpha2 = M_PI/2.0 - 0.0194; // RN 20180218A1
// const double DH_alpha2 = 1.5509; // RN 20180219A1 GOOD
// const double DH_alpha2 = 1.5439; // RN 20180222A1
// const double DH_alpha2 = 1.5491; // RN 20180319A1 BAD
// const double DH_alpha2 = 1.5507; // RN 20180319A2 

// const double DH_alpha2 = 1.5585; // RN 20180319PSM2A1

// prismatic axis is aligned with tool-shaft spin axis
const double DH_alpha3 = 0.0;
const double DH_alpha4 = M_PI/2.0;
// offset from wrist bend to jaw bend axis--> alpha is defined -pi/2
const double DH_alpha5 = -M_PI/2.0;
// choose gripper frame to have z-axis pointing along jaws
const double DH_alpha6 = M_PI/2.0;
// relevant only if treat jaws as separate links
const double DH_alpha7 = 0.0;

// the long and short distance from the portal to the exit of the cannula.
const double cannula_long_length = .078;
const double cannula_short_length = .035;

// q_Davinci vec: starts counting from 0; 0 displacement at model "home"
// when model joint_states are all at displacement 0.0
// use these offsets to convert to DH coords
// must command this much displacement to get wrist-bend axis to intersect base origin
const double insertion_offset = 0.0156;

const double DH_q_offset0 = 0.0;

// THETAs
const double DH_q_offset1 = M_PI/2.0;
// const double DH_q_offset1 = M_PI/2.0 - 0.0182; // RN 20180218 A1
// const double DH_q_offset1 = 1.5663; // RN 20180219A1
// const double DH_q_offset1 = 1.5710; // RN 20180222A1 GOOD
// const double DH_q_offset1 = 1.5538; // RN 20180319A1 BAD
// const double DH_q_offset1 = 1.5699; // RN 20180319A2 

// const double DH_q_offset1 = 1.5919; // RN 20180319PSM2A1 



// erdem IK tested, sign is negative. q3 should be larger than abs(insertion_offset)
const double DH_q_offset2 = -insertion_offset;
const double DH_q_offset3 = M_PI;
const double DH_q_offset4 = M_PI/2;
// M_PI;
const double DH_q_offset5 = M_PI/2;
// M_PI;
const double DH_q_offset6 = 0.0;

const double deg2rad = M_PI/180.0;


// deg2rad*45; //141; //51;
const double DH_q_max0 = 1.0;
// deg2rad*45;
const double DH_q_max1 = 0.7;
// 0.5;
const double DH_q_max2 = 0.23;
// deg2rad*180;
const double DH_q_max3 = 2.25;
// deg2rad*90;
const double DH_q_max4 = 1.57;
// deg2rad*90;
const double DH_q_max5 = 1.39;
// deg2rad*90;
const double DH_q_max6 = 1.57;

// -deg2rad*45; //51; //141;
const double DH_q_min0 = -1.0;
// -deg2rad*45;
const double DH_q_min1 = -0.7;
const double DH_q_min2 =  0.01;
// -deg2rad*180;
const double DH_q_min3 = -2.25;
// -deg2rad*90;
const double DH_q_min4 = -1.57;
// -deg2rad*90;
const double DH_q_min5 = -1.39;
// -deg2rad*90;
const double DH_q_min6 = -1.57;


// derived external defs:
const double dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis = 0.0091;

const double gripper_jaw_length = 0.0102;

const double DH_a_params[7] =
{
  DH_a1, DH_a2, DH_a3, DH_a4, DH_a5, DH_a6, DH_a7
};

// const double DH_d_params[7] =
// {
//   DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6, DH_d7
// };

const double DH_alpha_params[7] =
{
  DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6, DH_alpha7
};

const double DH_q_offsets[7] =
{
  DH_q_offset0, DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6
};

const double q_lower_limits[7] =
{
  DH_q_min0, DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6
};

const double q_upper_limits[7] =
{
  DH_q_max0, DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6
};

}  // namespace davinci_kinematics
