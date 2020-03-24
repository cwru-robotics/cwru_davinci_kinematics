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

#ifndef CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_H
#define CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_H

/**
 * @brief This list of constants are used for completing both the forward and inverse kinematic evaluation of a 
 * dvrk system.
 *
 * These measurements are also available from one of JHU's  dvrk website:
 * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/psm-large-needle-driver.json
 *
 * The kinematic models used in this library are based on DH parameters rather than joint location/direction as defined in the URDF standard.
 * However, the joint limits are the same as in the .xarco model files located in https://github.com/cwru-davinci/uv_geometry.
 */
namespace davinci_kinematics
{

/**
 * @brief This distance is measured to be between the gripper jaw and the gripper wrist.
 */
extern const double dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis;

/**
 * @brief This is the length of the gripper jaw. This is the part that actually grasps or cuts.
 */
extern const double gripper_jaw_length;

/**
 * @brief This is the list of Denavit–Hartenberg (DH) parameters which are often used for robot kinematic chains.
 * 
 * For further information about DH parameters, please consult https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
 */
extern const double DH_a_params[7];
extern const double DH_d_params[7];
extern const double DH_alpha_params[7];
extern const double DH_q_offsets[7];

/**
 * @brief These are the lower and upper limits of the dvrk Joints.
 */
extern const double q_lower_limits[7];
extern const double q_upper_limits[7];

/**
 * @brief The length from the portal to the edge of the cannula
 *
 * Long and short lengths.
 */
extern const double cannula_long_length;
extern const double cannula_short_length;

}  // namespace davinci_kinematics

#endif  // CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_H
