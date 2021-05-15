/*
 *  davinci_kinematic_definitions.cuh
 *  Copyright (C) 2017  Wyatt S. Newman, Russell C. Jackson, and Tom Shkurti.
 *  Copyright (C) 2021 Ethan Shafer.
 *  
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

#ifndef CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_CUDA_H
#define CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_CUDA_H
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

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
namespace davinci_kinematics_cuda
{
	//this defines the constants for the device
	#ifdef __CUDA_ARCH__
		__constant__
		const double PI = 3.14159265358979323846;
	
		__constant__
		const double cannula_long_length = .078;
		
		__constant__
		const double cannula_short_length = .035;
	
		__constant__
		const double insertion_offset = 0.0156;
		
		__constant__
		const double dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis = 0.0091;
	
		__constant__
		const double gripper_jaw_length = 0.0102;
	
		__constant__
		const double DH_a_params[7] = { 0.0, 0.0, 0.0, 0.0, dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis, 0.0, 0.0 };
	
		__constant__
		const double DH_alpha_params[7] = { PI/2.0, PI/2.0, 0.0, PI/2.0, -PI/2.0, PI/2.0, 0.0 };
	
		__constant__
		const double DH_q_offsets[7] = { 0.0, PI/2.0, -insertion_offset, PI, PI/2.0, PI/2.0, 0.0 };
	
		__constant__
		const double q_lower_limits[7] = { -PI/2.0, -0.7, 0.05, -3.0485, -PI/2.0, -1.39, -PI/2.0 };
	
		__constant__
		const double q_upper_limits[7] = { PI/2.0, 0.7, 0.23, 3.0485, PI/2.0, 1.39, PI/2.0 };
	// this defines the constants for the host
	#else
		const double PI = 3.14159265358979323846;
	
		const double cannula_long_length = .078;
		
		const double cannula_short_length = .035;
	
		const double insertion_offset = 0.0156;
		
		const double dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis = 0.0091;
	
		const double gripper_jaw_length = 0.0102;
	
		const double DH_a_params[7] = { 0.0, 0.0, 0.0, 0.0, dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis, 0.0, 0.0 };
	
		const double DH_alpha_params[7] = { PI/2.0, PI/2.0, 0.0, PI/2.0, -PI/2.0, PI/2.0, 0.0 };
	
		const double DH_q_offsets[7] = { 0.0, PI/2.0, -insertion_offset, PI, PI/2.0, PI/2.0, 0.0 };
	
		const double q_lower_limits[7] = { -PI/2.0, -0.7, 0.05, -3.0485, -PI/2.0, -1.39, -PI/2.0 };
	
		const double q_upper_limits[7] = { PI/2.0, 0.7, 0.23, 3.0485, PI/2.0, 1.39, PI/2.0 };		
	#endif

}  // namespace davinci_kinematics

#endif  // CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_H
