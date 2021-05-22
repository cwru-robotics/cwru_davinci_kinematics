/*
 *  davinci_fwd_kinematics.cu
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

#include <davinci_kinematics_cuda/davinci_fwd_kinematics.cuh>

namespace davinci_kinematics_cuda {
	
	// use member fncs to compute and multiply successive transforms
	__host__ __device__
	Forward::Forward() {
		// affine describing frame0 w/rt base frame
		Eigen::Matrix3d R_0_wrt_base;
		Eigen::Vector3d Origin_0_wrt_base;
		Origin_0_wrt_base << 0, 0, 0;

		// choose x0 to point down, so will not have a joint-angle offset for pitch
		Eigen::Vector3d x_axis(0, 0, -1);
		// consistent triad
		Eigen::Vector3d y_axis(1, 0, 0);
		// points IN, so + rotation is consistent leaning to the robot's left
		Eigen::Vector3d z_axis(0, -1, 0);

		// R_{0/base} = [0  1  0
		//               0  0 -1
		//              -1  0  0 ]
		R_0_wrt_base.col(0) = x_axis;
		R_0_wrt_base.col(1) = y_axis;
		R_0_wrt_base.col(2) = z_axis;

		this->affine_frame0_wrt_base_.linear() = R_0_wrt_base;
		this->affine_frame0_wrt_base_.translation() = Origin_0_wrt_base;

		// fill in a static tool transform from frame6 to a frame of interest on the gripper
		set_gripper_jaw_length(davinci_kinematics_cuda::gripper_jaw_length);

		for (int i = 0; i < 7; i++) {
			this->theta_DH_offsets_[i] = davinci_kinematics_cuda::DH_q_offsets[i];

		}
		// don't put prismatic displacement here
		this->theta_DH_offsets_[2] = 0.0;
		
		
		this->dval_DH_offsets_ << 0, 0, davinci_kinematics_cuda::DH_q_offsets[2], 0, 0, 0, 0; // RN 20180713 EXPERIMENTAL

		resetDhGenericParams();
	}
	
	__host__ __device__
	void Forward::resetDhGenericParams() {
		for (int i = 0; i < 7; i++) {
			this->theta_DH_offsets_generic_[i] = davinci_kinematics_cuda::DH_q_offsets[i];
			this->DH_alpha_params_generic_[i] = davinci_kinematics_cuda::DH_alpha_params[i];
			this->DH_a_params_generic_[i] = davinci_kinematics_cuda::DH_a_params[i];
			if (i == 2) {
				this->theta_DH_offsets_generic_[i] = 0;
			}
		}

		this->dval_DH_offsets_generic_ << 0, 0, davinci_kinematics_cuda::DH_q_offsets[2], 0, 0, 0, 0;
		this->j1_scale_factor_generic_ = 1.0;
		this->j2_scale_factor_generic_ = 1.0;
		this->j3_scale_factor_generic_ = 1.0;
	}

	// given a vector of joint states in DaVinci coords, convert these into
	// equivalent DH parameters, theta and d
	__host__ __device__
	void Forward::convert_qvec_to_DH_vecs(const Vector7 &q_vec, Vector7 &thetas_DH_vec, Vector7 &dvals_DH_vec) {
		// +? -?
		thetas_DH_vec = this->theta_DH_offsets_;
		for (int i = 0; i < 7; i++) {
			// skip the linear joint.
			if (i == 2)
				continue;
			thetas_DH_vec(i) += q_vec(i);
		}

		dvals_DH_vec = this->dval_DH_offsets_;
		dvals_DH_vec(2) += q_vec(2); // RN original
	}

	__host__ __device__
	double Forward::dh_var_to_qvec(double dh_val, int index) {
		return (dh_val - davinci_kinematics_cuda::DH_q_offsets[index]);
	}

	// given 4 DH parameters, compute the corresponding transform as an affine3d
	__host__ __device__
	Eigen::Affine3d Forward::computeAffineOfDH(double a, double d, double alpha, double theta) {
		Eigen::Affine3d affine_DH;
		Eigen::Matrix3d R;
		Eigen::Vector3d p;

		double cq = cos(theta);
		double sq = sin(theta);
		double sa = sin(alpha);
		double ca = cos(alpha);
		R(0, 0) = cq;

		// - sin(q(i))*cos(alpha);
		R(0, 1) = -sq * ca;

		// sin(q(i))*sin(alpha);
		R(0, 2) = sq * sa;
		R(1, 0) = sq;
		// cos(q(i))*cos(alpha);
		R(1, 1) = cq * ca;
		R(1, 2) = -cq * sa;

		R(2, 0) = 0;
		R(2, 1) = sa;
		R(2, 2) = ca;
		affine_DH.linear() = R;

		p(0) = a * cq;
		p(1) = a * sq;
		p(2) = d;
		affine_DH.translation() = p;

		return affine_DH;
	}

	// provide DH theta and d values, return affine pose of gripper tip w/rt base frame
	// also computes all intermediate affine frames, w/rt base frame
	__host__ __device__
	void Forward::fwd_kin_solve_DH(const Vector7 &theta_vec, const Vector7 &d_vec, const unsigned int joint,
			Eigen::Affine3d &result) {
		// use or affect these member variables:
		Eigen::Affine3d affines_i_wrt_iminus1[7];
		Eigen::Affine3d affines_products[7];
		Eigen::Affine3d transform;
		
		double a, d, theta, alpha;

		for (int i = 0; i < 7; i++) {
			a = DH_a_params[i];
			d = d_vec(i);
			alpha = DH_alpha_params[i];
			theta = theta_vec(i);
			transform = computeAffineOfDH(a, d, alpha, theta);
			affines_i_wrt_iminus1[i] = transform;
		}
		
		affines_products[0] = this->affine_frame0_wrt_base_ * affines_i_wrt_iminus1[0];
		// RN Note that it starts from 1.
		for (int i = 1; i <= joint; i++) {
			affines_products[i] = affines_products[i - 1] * affines_i_wrt_iminus1[i];
		}
		result = affines_products[joint];
	}
	
	__host__ __device__
	Eigen::Affine3d Forward::fwd_kin_solve(const Vector7 &q_vec, const unsigned int desired_joint) {
		unsigned int joint = desired_joint - 1;
		Vector7 thetas_DH_vec;
		Vector7 dvals_DH_vec;
		convert_qvec_to_DH_vecs(q_vec, thetas_DH_vec, dvals_DH_vec);
		
		Eigen::Affine3d result;
		fwd_kin_solve_DH(thetas_DH_vec, dvals_DH_vec, joint, result);
		return result;
	}

	__host__ __device__
	Eigen::Affine3d Forward::fwd_kin_solve(const double *q_vec, const unsigned int desired_joint) {
		Vector7 q;
		q[0] = q_vec[0];
		q[1] = q_vec[1];
		q[2] = q_vec[2];
		q[3] = q_vec[3];
		q[4] = q_vec[4];
		q[5] = q_vec[5];
		q[6] = q_vec[6];
		return this->fwd_kin_solve(q, desired_joint);
	}

	__host__ __device__
	void Forward::set_gripper_jaw_length(double jaw_length) {
		this->gripper_jaw_length_ = jaw_length;
		this->affine_gripper_wrt_frame6_ = computeAffineOfDH(0, this->gripper_jaw_length_, 0, -davinci_kinematics_cuda::PI / 2);
	}

}  // namespace davinci_kinematics
