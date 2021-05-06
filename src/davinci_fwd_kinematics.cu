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

		this->theta_DH_offsets_.resize(7);
		for (int i = 0; i < 7; i++) {
			this->theta_DH_offsets_(i) = davinci_kinematics_cuda::DH_q_offsets[i];

		}
		// don't put prismatic displacement here
		this->theta_DH_offsets_(2) = 0.0;

		this->dval_DH_offsets_.resize(7);
		// dval_DH_offsets_<< 0, 0 , DH_q_offsets[2], 0, 0, 0, 0;

		//dval_DH_offsets_<< 0, 0, 0, 0, 0, 0, 0; // RN 20180712A1 PSM1
		this->dval_DH_offsets_ << 0, 0, davinci_kinematics_cuda::DH_q_offsets[2], 0, 0, 0, 0; // RN 20180713 EXPERIMENTAL

		// resize MatrixXd Jacobian_ and initialize terms to 0's
		this->Jacobian_ = Eigen::MatrixXd::Zero(6, 6);

		resetDhGenericParams();
	}

	// fnc to extract a joint value from a JointState message;
	// provide the name of interest, as a C++ string, and provide the entire
	// jointState message;  will set the value of "qval" arg, if possible;
	// will return "true" or "false" to indicate if name was found on list
	/*bool Forward::get_jnt_val_by_name(std::string jnt_name, sensor_msgs::JointState jointState, double &qval) {
		//TODO replace sensor_msgs::JointState with appropriate data type

	}*/

	// given a vector of joint states in DaVinci coords, convert these into
	// equivalent DH parameters, theta and d
	__host__ __device__
	void Forward::convert_qvec_to_DH_vecs(const Vectorq7x1 &q_vec, Eigen::VectorXd &thetas_DH_vec, Eigen::VectorXd &dvals_DH_vec) {
		thetas_DH_vec.resize(7);
		// +? -?
		thetas_DH_vec = this->theta_DH_offsets_;
		for (int i = 0; i < 7; i++) {
			// skip the linear joint.
			if (i == 2)
				continue;
			thetas_DH_vec(i) += q_vec(i);
		}

		dvals_DH_vec.resize(7);
		dvals_DH_vec = this->dval_DH_offsets_;
		dvals_DH_vec(2) += q_vec(2); // RN original
	}

	__host__ __device__
	int Forward::check_jnts(const Vectorq7x1 &q_vec) {
		int result(0);
		for (int i(0); i < 6; i++) {
			if (q_vec(i) < davinci_kinematics_cuda::q_lower_limits[i] || q_vec(i) > davinci_kinematics_cuda::q_upper_limits[i]) {
				result -= (1 << i);
			}
		}
		if (result < 0) {
			return result;
		}
		// The wrist is inside the cannula.
		if (q_vec(2) < davinci_kinematics_cuda::cannula_short_length) {
			// if the wrist is straight, then it is ok.
			for (unsigned int i(3); i < 6; i++) {
				if (abs(q_vec(i)) > 0.001) {
					return 1;
				}
			}
		}
		// Test if the gripper is open wider than available.
		if (abs(q_vec(6)) > (davinci_kinematics_cuda::PI - abs(q_vec(5)))) {
			return 2;
		}
		return 0;
	}

	__host__ __device__
	double Forward::dh_var_to_qvec(double dh_val, int index) {
		return (dh_val - davinci_kinematics_cuda::DH_q_offsets[index]);
	}

	// RN TODO deal with that .99 sacle factor
	//    Eigen::VectorXd thetas_DH_vec_,dvals_DH_vec_;
	__host__ __device__
	Vectorq7x1 Forward::convert_DH_vecs_to_qvec(const Eigen::VectorXd &thetas_DH_vec, const Eigen::VectorXd &dvals_DH_vec) {
		Vectorq7x1 q_vec;

		for (int i = 0; i < 7; i++) {
			q_vec(i) = thetas_DH_vec(i) - this->theta_DH_offsets_(i);
		}
		q_vec(2) = dvals_DH_vec(2) - this->dval_DH_offsets_(2);

		return q_vec;
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
	void Forward::fwd_kin_solve_DH(const Eigen::VectorXd &theta_vec, const Eigen::VectorXd &d_vec) {
		// use or affect these member variables:

		Eigen::Affine3d *affines_i_wrt_iminus1 = (Eigen::Affine3d*) malloc(7 * sizeof(Eigen::Affine3d));
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

		this->affine_products_ = (Eigen::Affine3d*) malloc(7 * sizeof(double));
		this->affine_products_[0] = this->affine_frame0_wrt_base_ * affines_i_wrt_iminus1[0];
		// RN Note that it starts from 1.
		for (int i = 1; i < 7; i++) {
			this->affine_products_[i] = this->affine_products_[i - 1] * affines_i_wrt_iminus1[i];
		}
		this->affine_gripper_wrt_base_ = this->affine_products_[6] * affine_gripper_wrt_frame6_;

		// RN added for wrist pt coordinate w/rt base frame
		this->affine_wrist_wrt_base_ = this->affine_products_[2];
	}

	__host__ __device__
	Eigen::Affine3d Forward::get_wrist_wrt_base() // RN
	{
		return this->affine_wrist_wrt_base_;
	}
	
	__host__ __device__
	Eigen::Affine3d Forward::fwd_kin_solve(const Vectorq7x1 &q_vec, const unsigned int desired_joint) {
		unsigned int joint = desired_joint - 1;
		this->current_joint_state_ = q_vec;
		Eigen::VectorXd thetas_DH_vec, dvals_DH_vec;
		convert_qvec_to_DH_vecs(q_vec, thetas_DH_vec, dvals_DH_vec);
		fwd_kin_solve_DH(thetas_DH_vec, dvals_DH_vec);

		if (joint < 6) {
			return this->affine_products_[joint];
		} else {
			return this->affine_gripper_wrt_base_;
		}
	}

	__host__ __device__
	Eigen::Affine3d Forward::fwd_kin_solve(const double *q_vec, const unsigned int desired_joint) {
		Vectorq7x1 q;
		q[0] = q_vec[0];
		q[1] = q_vec[1];
		q[2] = q_vec[2];
		q[3] = q_vec[3];
		q[4] = q_vec[4];
		q[5] = q_vec[5];
		q[6] = q_vec[6];
		return this->fwd_kin_solve(q, desired_joint);
	}

	/*
	Eigen::Affine3d Forward::fwd_kin_solve(const sensor_msgs::JointState &jointStateMsg, const unsigned int desiredJoint) {

		//TODO:  Add error checking to ensure jointStateMsg has proper values
		Vectorq7x1 q;

		for (unsigned int i = 0; i < jointStateMsg.name.size(); i++) {
			unsigned int position = davinci_kinematics::jointNameToOrder.find(jointStateMsg.name[i])->second;
			q[position] = jointStateMsg.position[i];
		}

		return this->fwd_kin_solve(q, desiredJoint);
	}*/

	__host__ __device__
	Eigen::Affine3d Forward::fwd_kin_solve() {
		return this->affine_gripper_wrt_base_;
	}

	__host__ __device__
	Eigen::Affine3d Forward::get_frame0_wrt_base() const {
		return this->affine_frame0_wrt_base_;
	}

	__host__ __device__
	void Forward::set_frame0_wrt_base(const Eigen::Affine3d &affine_frame0_wrt_base) {
		this->affine_frame0_wrt_base_ = affine_frame0_wrt_base;
	}

	__host__ __device__
	Eigen::Affine3d Forward::get_gripper_wrt_frame6() const {
		return this->affine_gripper_wrt_frame6_;
	}

	__host__ __device__
	void Forward::set_gripper_jaw_length(double jaw_length) {
		this->gripper_jaw_length_ = jaw_length;
		this->affine_gripper_wrt_frame6_ = computeAffineOfDH(0, this->gripper_jaw_length_, 0, -davinci_kinematics_cuda::PI / 2);
	}

	__host__ __device__
	Eigen::MatrixXd Forward::compute_jacobian(const Vectorq7x1 &q_vec) {
		// use the jacobian to make the computation.
		fwd_kin_solve(q_vec);
		Eigen::Vector3d z_axis;
		Eigen::Vector3d vec_tip_minus_Oi_wrt_base;
		Eigen::Matrix3d R;
		Eigen::Vector3d r_tip_wrt_base = this->affine_gripper_wrt_base_.translation();
		Eigen::Vector3d z_axis0 = this->affine_frame0_wrt_base_.linear().col(2);
		// angular Jacobian is just the z axes of each revolute joint (expressed in base frame);
		// for prismatic joint, there is no angular contribution
		// start from z_axis0

		// Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
		this->Jacobian_.block<3, 1>(3, 0) = z_axis0;
		vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - this->affine_frame0_wrt_base_.translation();
		this->Jacobian_.block<3, 1>(0, 0) = z_axis0.cross(vec_tip_minus_Oi_wrt_base);
		// 2nd joint:
		// refer to previous joint's z axis
		R = this->affine_products_[0].linear();
		z_axis = R.col(2);
		// Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
		this->Jacobian_.block<3, 1>(3, 1) = z_axis;
		vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - this->affine_products_[0].translation();
		this->Jacobian_.block<3, 1>(0, 1) = z_axis.cross(vec_tip_minus_Oi_wrt_base);

		// prismatic joint:
		R = this->affine_products_[1].linear();
		z_axis = R.col(2);
		this->Jacobian_.block<3, 1>(0, 2) = z_axis;

		// joints 4-6:
		for (int i = 3; i < 6; i++) {
			R = this->affine_products_[i - 1].linear();
			z_axis = R.col(2);
			// Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
			this->Jacobian_.block<3, 1>(3, i) = z_axis;
			vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - this->affine_products_[i - 1].translation();
			this->Jacobian_.block<3, 1>(0, i) = z_axis.cross(vec_tip_minus_Oi_wrt_base);
		}
		// translational Jacobian depends on joint's z-axis and vector from i'th axis to robot tip
		return this->Jacobian_;
	}

	__host__ __device__
	Eigen::MatrixXd Forward::compute_jacobian() {
		return this->Jacobian_;
	}

	__host__ __device__
	void Forward::resetDhGenericParams() {
		this->theta_DH_offsets_generic_.resize(7);
		this->dval_DH_offsets_generic_.resize(7);
		this->DH_a_params_generic_.resize(7);
		this->DH_alpha_params_generic_.resize(7);

		for (int i = 0; i < 7; i++) {
			this->theta_DH_offsets_generic_(i) = davinci_kinematics_cuda::DH_q_offsets[i];
			this->DH_alpha_params_generic_(i) = davinci_kinematics_cuda::DH_alpha_params[i];
			this->DH_a_params_generic_(i) = davinci_kinematics_cuda::DH_a_params[i];
			if (i == 2) {
				this->theta_DH_offsets_generic_(i) = 0;
			}
		}

		this->dval_DH_offsets_generic_ << 0, 0, davinci_kinematics_cuda::DH_q_offsets[2], 0, 0, 0, 0;
		this->j1_scale_factor_generic_ = 1.0;
		this->j2_scale_factor_generic_ = 1.0;
		this->j3_scale_factor_generic_ = 1.0;
	}
}  // namespace davinci_kinematics
