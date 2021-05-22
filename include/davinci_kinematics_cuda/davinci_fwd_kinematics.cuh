/*
 *  davinci_fwd_kinematics.cuh
 *  Copyright (C) 2017  Wyatt S. Newman, Russell C. Jackson, and Tom Shkurti.
 *  Copyright (C) 2021 Ethan Shafer.
 *  
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// NOTE:  FK and IK assume that the gripper-tip frame is expressed with respect
// to the respective PMS base frame (not camera frame).  For motions w/rt camera
//  first transform the desired camera-frame pose into base-frame pose.
#ifndef CWRU_DAVINCI_KINEMATICS_DAVINCI_FWD_KINEMATICS_CUDA_H
#define CWRU_DAVINCI_KINEMATICS_DAVINCI_FWD_KINEMATICS_CUDA_H

#include <davinci_kinematics_cuda/davinci_kinematic_definitions.cuh>

//TODO update so that anyone can use the development branch with intelligent path referencing
#include </home/ethan/catkin_ws/src/eigen/Eigen/Eigen>

#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

/**
 * @brief The davinci_kinematics namespace is where both the forward and inverse dvrk kinematic libraries are defined
 *
 * This is to prevent namespace collisions.
 */
namespace davinci_kinematics_cuda {

	/**
	 * @brief The Vectorq7x1 is used to save space and act as the SE(3) transform space and jointspace states respectively.
	 */
	typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;
	typedef Eigen::Vector<double, 7> Vector7;

	/**
	 * @brief The Forward kinematics library definition.
	 *
	 * In addition to computing the SE(3) transform based on a set of joint angles,
	 * This object also generates a Jacobian matrix.
	 */
	class Forward {
		public:

			/**
			 * @brief Get the value of a joint angle by name
			 *
			 *
			 */
			//static bool get_jnt_val_by_name(std::string jnt_name, sensor_msgs::JointState jointState, double &qval);

			/**
			 * @brief The default constructor of the forward kinematics library.
			 */
			__host__ __device__
			Forward();

			/**
			 * @brief Given the full set of joint positions, compute the SE(3) transform of the  Provide joint angles and prismatic displacement (q_vec[2]) w/rt DaVinci coords
			 *
			 * will get translated to DH coords to solve fwd kin
			 * return affine describing gripper pose w/rt base frame
			 *
			 * @param q_vec (optional) The joint states of the robot.
			 *
			 * @return The full DaVinci SE(3) transform.
			 */
			__host__ __device__
			Eigen::Affine3d fwd_kin_solve(const Vector7 &q_vec, const unsigned int desired_joint = 7);
			
			__host__ __device__
			Eigen::Affine3d fwd_kin_solve(const double *q_vec, const unsigned int desired_joint = 7);

			/**
			 * @brief sets the transform from the joint frame 6 frame to the gripper frame.
			 *
			 * @param jaw_length the length of the gripper jaw.
			 *
			 * @return The transform between the joint frame 6 and the gripper frame.
			 */
			__host__ __device__
			void set_gripper_jaw_length(double jaw_length);

			__host__ __device__
			void resetDhGenericParams();

		protected:
			/**
			 * @brief The following function takes args of DH thetas and d's and returns an affine transformation
			 *
			 * @param theta_vec The vector of DH theta values.
			 * @param d_vec The vector of DH q values.
			 *
			 * @return The full DaVinci SE(3) transform.
			 */
			__host__ __device__
			void fwd_kin_solve_DH(const Vector7 &theta_vec, const Vector7 &d_vec, const unsigned int joint,
					Eigen::Affine3d &result);

			// RN
			Vector7 theta_DH_offsets_generic_;
			Vector7 dval_DH_offsets_generic_;
			Vector7 DH_a_params_generic_;
			Vector7 DH_alpha_params_generic_;
			double j1_scale_factor_generic_;
			double j2_scale_factor_generic_;
			double j3_scale_factor_generic_;

		private:
			/**
			 * @brief Given a vector of joint states in DaVinci coords, convert these into
			 * equivalent DH parameters, theta and d
			 *
			 * @param q_vec the vector of joint states.
			 *
			 * @param thetas_DH_vec the resulting DH theta parameters.
			 * @param dvals_DH_vec the resulting d values of the theta parameters.
			 */
			__host__ __device__
			void convert_qvec_to_DH_vecs(const Vector7 &q_vec, Vector7 &thetas_DH_vec, Vector7 &dvals_DH_vec);

			/**
			 * @brief Given a full set of DH parameters, compute the Eigen Affine transform.
			 *
			 * This Affine transform is also a member of the SE(3) subspace of affine transforms.
			 *
			 * @param a The a value of the DH parameter.
			 * @param d The d value of the DH parameter.
			 * @param alpha The alpha value of the DH parameter.
			 * @param theta The theta value of the DH parameter.
			 *
			 * @return The SE(3) transformation computed from the input DH parameters.
			 */
			__host__ __device__
			Eigen::Affine3d computeAffineOfDH(double a, double d, double alpha, double theta);

			/**
			 * @brief convert the dh  joint parameter to the joint value for a specific joint index.
			 *
			 * @param dh_val the dh value
			 * @param index The index of the joint (0-6)
			 *
			 * @return the joint value
			 */
			__host__ __device__
			double dh_var_to_qvec(double dh_val, int index);

			/**
			 * @brief This is the tranform aligning the robot with a world frame.
			 */
			Eigen::Affine3d affine_frame0_wrt_base_;

			/**
			 * @brief This is the tranform between the last joint and the robot gripper tip.
			 */
			Eigen::Affine3d affine_gripper_wrt_frame6_;

			/**
			 * @brief This is the tranform between the gripper and the robot base.
			 *
			 * recomputed when a new set of joints are passed in.
			 */
			Eigen::Affine3d affine_gripper_wrt_base_;

			Eigen::Affine3d affine_wrist_wrt_base_;

			/**
			 * @brief This is the list of theta offset values for the DaVinci DH joint parameters.
			 *
			 * Stored intrinsicaly as part of the dvrk kinematics
			 */
			Vector7 theta_DH_offsets_;

			/**
			 * @brief This is the list of d offset values for the DaVinci DH joint parameters.
			 *
			 * Stored intrinsicaly as part of the dvrk kinematics
			 */
			Vector7 dval_DH_offsets_;

			/**
			 * @brief the length of the gripper jaw.
			 */
			double gripper_jaw_length_;
	};

}  // namespace davinci_kinematics

#endif  // CWRU_DAVINCI_KINEMATICS_DAVINCI_FWD_KINEMATICS_H
