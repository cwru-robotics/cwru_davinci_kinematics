/*
 *  davinci_fwd_kinematics.cpp
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

#include <davinci_kinematics/davinci_fwd_kinematics.h>
#include <math.h>
#include <string>
#include <vector>

namespace davinci_kinematics {

	// fnc to extract a joint value from a JointState message;
	// provide the name of interest, as a C++ string, and provide the entire
	// jointState message;  will set the value of "qval" arg, if possible;
	// will return "true" or "false" to indicate if name was found on list
	bool Forward::get_jnt_val_by_name(std::string jnt_name, sensor_msgs::JointState jointState, double &qval) {
		int njnts = jointState.name.size();
		for (int ijnt = 0; ijnt < njnts; ijnt++) {
			if (jnt_name.compare(jointState.name[ijnt]) == 0) {
				// found a name match!
				qval = jointState.position[ijnt];
				return true;
			}
		}
		// if get to here, did not find a match:
		printf("no match for joint name %s\n", jnt_name.c_str());
		return false;
	}

	// given a vector of joint states in DaVinci coords, convert these into
	// equivalent DH parameters, theta and d
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

	int Forward::check_jnts(const Vectorq7x1 &q_vec) {
		int result(0);
		for (int i(0); i < 6; i++) {
			if (q_vec(i) < q_lower_limits[i] || q_vec(i) > q_upper_limits[i]) {
				result -= (1 << i);
			}
		}
		if (result < 0) {
			return result;
		}
		// The wrist is inside the cannula.
		if (q_vec(2) < cannula_short_length) {
			// if the wrist is straight, then it is ok.
			for (unsigned int i(3); i < 6; i++) {
				if (abs(q_vec(i)) > 0.001) {
					return 1;
				}
			}
		}
		// Test if the gripper is open wider than available.
		if (abs(q_vec(6)) > (M_PI - abs(q_vec(5)))) {
			return 2;
		}
		return 0;
	}

	double Forward::dh_var_to_qvec(double dh_val, int index) {
		return (dh_val - DH_q_offsets[index]);
	}

	// RN TODO deal with that .99 sacle factor
	//    Eigen::VectorXd thetas_DH_vec_,dvals_DH_vec_;
	Vectorq7x1 Forward::convert_DH_vecs_to_qvec(const Eigen::VectorXd &thetas_DH_vec, const Eigen::VectorXd &dvals_DH_vec) {
		Vectorq7x1 q_vec;

		for (int i = 0; i < 7; i++) {
			q_vec(i) = thetas_DH_vec(i) - this->theta_DH_offsets_(i);
		}
		q_vec(2) = dvals_DH_vec(2) - this->dval_DH_offsets_(2);

		return q_vec;
	}

	// given 4 DH parameters, compute the corresponding transform as an affine3d
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

	// use member fncs to compute and multiply successive transforms
	Forward::Forward() {
		// affine describing frame0 w/rt base frame--see comments above
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
		set_gripper_jaw_length(gripper_jaw_length);

		this->theta_DH_offsets_.resize(7);
		for (int i = 0; i < 7; i++) {
			this->theta_DH_offsets_(i) = DH_q_offsets[i];

		}
		// don't put prismatic displacement here
		this->theta_DH_offsets_(2) = 0.0;

		this->dval_DH_offsets_.resize(7);
		// dval_DH_offsets_<< 0, 0 , DH_q_offsets[2], 0, 0, 0, 0;

		//dval_DH_offsets_<< 0, 0, 0, 0, 0, 0, 0; // RN 20180712A1 PSM1
		this->dval_DH_offsets_ << 0, 0, DH_q_offsets[2], 0, 0, 0, 0; // RN 20180713 EXPERIMENTAL

		// resize MatrixXd Jacobian_ and initialize terms to 0's
		this->Jacobian_ = Eigen::MatrixXd::Zero(6, 6);

		resetDhGenericParams();
	}

	// provide DH theta and d values, return affine pose of gripper tip w/rt base frame
	// also computes all intermediate affine frames, w/rt base frame
	void Forward::fwd_kin_solve_DH(const Eigen::VectorXd &theta_vec, const Eigen::VectorXd &d_vec) {
		// use or affect these member variables:

		std::vector<Eigen::Affine3d> affines_i_wrt_iminus1(7);
		Eigen::Affine3d transform;
		double a, d, theta, alpha;

		for (int i = 0; i < 7; i++) {
			a = davinci_kinematics::DH_a_params[i];
			d = d_vec(i);
			alpha = davinci_kinematics::DH_alpha_params[i];
			theta = theta_vec(i);
			transform = computeAffineOfDH(a, d, alpha, theta);
			affines_i_wrt_iminus1[i] = transform;
		}

		this->affine_products_.resize(7);
		this->affine_products_[0] = this->affine_frame0_wrt_base_ * affines_i_wrt_iminus1[0];
		// RN Note that it starts from 1.
		for (int i = 1; i < 7; i++) {
			this->affine_products_[i] = this->affine_products_[i - 1] * affines_i_wrt_iminus1[i];
		}
		this->affine_gripper_wrt_base_ = this->affine_products_[6] * affine_gripper_wrt_frame6_;

		// RN added for wrist pt coordinate w/rt base frame
		this->affine_wrist_wrt_base_ = this->affine_products_[2];
	}

	Eigen::Affine3d Forward::get_wrist_wrt_base() // RN
	{
		return this->affine_wrist_wrt_base_;
	}

	Eigen::Affine3d Forward::get_wrist_wrt_base(std::string kinematic_set_name) // RN
			{
		return this->affine_wrist_wrt_base_map_[kinematic_set_name];
	}

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

	Eigen::Affine3d Forward::fwd_kin_solve(const std::vector<double> &q_vec, const unsigned int desired_joint) {
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

	Eigen::Affine3d Forward::fwd_kin_solve(const sensor_msgs::JointState &jointStateMsg, const unsigned int desiredJoint) {

		//TODO:  Add error checking to ensure jointStateMsg has proper values
		Vectorq7x1 q;

		for (unsigned int i = 0; i < jointStateMsg.name.size(); i++) {
			unsigned int position = davinci_kinematics::jointNameToOrder.find(jointStateMsg.name[i])->second;
			q[position] = jointStateMsg.position[i];
		}

		return this->fwd_kin_solve(q, desiredJoint);
	}

	Eigen::Affine3d Forward::fwd_kin_solve() {
		return this->affine_gripper_wrt_base_;
	}

	Eigen::Affine3d Forward::get_frame0_wrt_base() const {
		return this->affine_frame0_wrt_base_;
	}

	void Forward::set_frame0_wrt_base(const Eigen::Affine3d &affine_frame0_wrt_base) {
		this->affine_frame0_wrt_base_ = affine_frame0_wrt_base;
	}

	Eigen::Affine3d Forward::get_gripper_wrt_frame6() const {
		return this->affine_gripper_wrt_frame6_;
	}

	void Forward::set_gripper_jaw_length(double jaw_length) {
		this->gripper_jaw_length_ = jaw_length;
		this->affine_gripper_wrt_frame6_ = computeAffineOfDH(0, this->gripper_jaw_length_, 0, -M_PI / 2);
	}

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

	Eigen::MatrixXd Forward::compute_jacobian() {
		return this->Jacobian_;
	}

// gen_rand_legal_jnt_vals: compute random values within legal joint range:
	void Forward::gen_rand_legal_jnt_vals(Vectorq7x1 &qvec) {
		qvec(6) = 0;
		double drand_val;
		unsigned int seed = time(NULL);
		for (int i = 0; i < 6; i++) {
			drand_val = static_cast<double>(rand_r(&seed)) / (static_cast<double>(RAND_MAX));
			qvec(i) = q_lower_limits[i] + (q_upper_limits[i] - q_lower_limits[i]) * drand_val;
		}
	}

// TODO
	bool Forward::loadDHyamlfiles(std::string yaml_name, std::string kinematic_set_name) {

		std::string psm_yaml_path;
		YAML::Node psm_dh_param_node;

		double j1_scale_factor;
		double j2_scale_factor;
		double j3_scale_factor;
		Eigen::VectorXd theta_DH_offsets;
		Eigen::VectorXd dval_DH_offsets;
		Eigen::VectorXd DH_a_params;
		Eigen::VectorXd DH_alpha_params;

		theta_DH_offsets.resize(7);
		dval_DH_offsets.resize(7);
		DH_a_params.resize(7);
		DH_alpha_params.resize(7);

		// Make them equal to generic ones first.
		theta_DH_offsets = this->theta_DH_offsets_generic_;
		dval_DH_offsets = this->dval_DH_offsets_generic_;
		DH_a_params = this->DH_a_params_generic_;
		DH_alpha_params = this->DH_alpha_params_generic_;
		j1_scale_factor = this->j1_scale_factor_generic_;
		j2_scale_factor = this->j2_scale_factor_generic_;
		j3_scale_factor = this->j3_scale_factor_generic_;

		this->ros_pkg_path_ = ros::package::getPath("cwru_davinci_kinematics");
		psm_yaml_path = this->ros_pkg_path_ + "/config/" + yaml_name + ".yaml";

		try {
			ROS_INFO("Loading PSM DH parameter yaml file...");
			psm_dh_param_node = YAML::LoadFile(psm_yaml_path);
		} catch (YAML::ParserException &e) {
			ROS_WARN("Failed to yaml.");
			std::cout << e.what() << std::endl;
			return false;
		}

		// Load thetas from yaml,

		try {
			theta_DH_offsets[0] = psm_dh_param_node["theta_1"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH theta_1, will use generic instead.");
		}

		try {
			theta_DH_offsets[1] = psm_dh_param_node["theta_2"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH theta_2, will use generic instead.");
		}

		try {
			theta_DH_offsets[2] = psm_dh_param_node["theta_3"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH theta_3, will use generic instead.");
		}

		try {
			theta_DH_offsets[3] = psm_dh_param_node["theta_4"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH theta_4, will use generic instead.");
		}

		try {
			theta_DH_offsets[4] = psm_dh_param_node["theta_5"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH theta_5, will use generic instead.");
		}

		// Load alphas from yaml.

		try {
			DH_alpha_params[0] = psm_dh_param_node["alpha_1"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH alpha_1, will use generic instead.");
		}

		try {
			DH_alpha_params[1] = psm_dh_param_node["alpha_2"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH alpha_2, will use generic instead.");
		}

		try {
			DH_alpha_params[2] = psm_dh_param_node["alpha_3"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH alpha_3, will use generic instead.");
		}

		try {
			DH_alpha_params[3] = psm_dh_param_node["alpha_4"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH alpha_4, will use generic instead.");
		}

		try {
			DH_alpha_params[4] = psm_dh_param_node["alpha_5"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH alpha_5, will use generic instead.");
		}

		// Load a from yaml.

		try {
			DH_a_params[0] = psm_dh_param_node["a_1"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH a_1, will use generic instead.");
		}

		try {
			DH_a_params[1] = psm_dh_param_node["a_2"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH a_2, will use generic instead.");
		}

		try {
			DH_a_params[2] = psm_dh_param_node["a_3"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH a_3, will use generic instead.");
		}

		try {
			DH_a_params[3] = psm_dh_param_node["a_4"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH a_4, will use generic instead.");
		}

		try {
			DH_a_params[4] = psm_dh_param_node["a_5"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH a_5, will use generic instead.");
		}

		// Load d from yaml.

		try {
			dval_DH_offsets[0] = psm_dh_param_node["d_1"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH d_1, will use generic instead.");
		}

		try {
			dval_DH_offsets[1] = psm_dh_param_node["d_2"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH d_2, will use generic instead.");
		}

		try {
			dval_DH_offsets[2] = psm_dh_param_node["d_3"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH d_3, will use generic instead.");
		}

		try {
			dval_DH_offsets[3] = psm_dh_param_node["d_4"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH d_4, will use generic instead.");
		}

		try {
			dval_DH_offsets[4] = psm_dh_param_node["d_5"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load DH d_5, will use generic instead.");
		}

		try {
			j1_scale_factor = psm_dh_param_node["j1_scale_factor"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load j1_scale_factor, will use generic instead.");
		}

		try {
			j2_scale_factor = psm_dh_param_node["j2_scale_factor"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load j2_scale_factor, will use generic instead.");
		}

		try {
			j3_scale_factor = psm_dh_param_node["j3_scale_factor"].as<double>();
		} catch (const std::exception &e) {
			std::cout << e.what() << std::endl;
			ROS_WARN("Failed to load j3_scale_factor, will use generic instead.");
		}

		ROS_INFO("Yaml loading complete!");

		// Add to maps
		this->theta_DH_offsets_map_[kinematic_set_name] = theta_DH_offsets;
		this->dval_DH_offsets_map_[kinematic_set_name] = dval_DH_offsets;
		this->DH_a_params_map_[kinematic_set_name] = DH_a_params;
		this->DH_alpha_params_map_[kinematic_set_name] = DH_alpha_params;
		this->j1_scale_factor_map_[kinematic_set_name] = j1_scale_factor;
		this->j2_scale_factor_map_[kinematic_set_name] = j2_scale_factor;
		this->j3_scale_factor_map_[kinematic_set_name] = j3_scale_factor;

		// Also initialise related variables if needed.

		return true;

	}

	void Forward::resetDhOffsetsMaps() {
		this->theta_DH_offsets_map_.clear();
		this->dval_DH_offsets_map_.clear();
		this->DH_a_params_map_.clear();
		this->DH_alpha_params_map_.clear();
		this->affine_gripper_wrt_base_map_.clear();
		this->affine_wrist_wrt_base_map_.clear();
		this->affine_products_map_.clear();
		this->current_joint_state__map_.clear();
		this->Jacobian_map_.clear();
	}

	void Forward::resetDhGenericParams() {
		this->theta_DH_offsets_generic_.resize(7);
		this->dval_DH_offsets_generic_.resize(7);
		this->DH_a_params_generic_.resize(7);
		this->DH_alpha_params_generic_.resize(7);

		for (int i = 0; i < 7; i++) {
			this->theta_DH_offsets_generic_(i) = DH_q_offsets[i];
			this->DH_alpha_params_generic_(i) = DH_alpha_params[i];
			this->DH_a_params_generic_(i) = DH_a_params[i];
			if (i == 2) {
				this->theta_DH_offsets_generic_(i) = 0;
			}
		}

		this->dval_DH_offsets_generic_ << 0, 0, DH_q_offsets[2], 0, 0, 0, 0;
		this->j1_scale_factor_generic_ = 1.0;
		this->j2_scale_factor_generic_ = 1.0;
		this->j3_scale_factor_generic_ = 1.0;
	}

	void Forward::printAllDhMaps() {
		std::cout << "theta_DH_offsets_map_ size: " << this->theta_DH_offsets_map_.size() << std::endl;
		for (auto it = this->theta_DH_offsets_map_.begin(); it != this->theta_DH_offsets_map_.end(); ++it) {
			std::cout << it->first << ":" << std::endl;
			std::cout << it->second.transpose() << std::endl;
		}

		std::cout << "dval_DH_offsets_map_ size: " << dval_DH_offsets_map_.size() << std::endl;
		for (auto it = dval_DH_offsets_map_.begin(); it != dval_DH_offsets_map_.end(); ++it) {
			std::cout << it->first << ":" << std::endl;
			std::cout << it->second.transpose() << std::endl;
		}

		std::cout << "DH_a_params_map_ size: " << DH_a_params_map_.size() << std::endl;
		for (auto it = DH_a_params_map_.begin(); it != DH_a_params_map_.end(); ++it) {
			std::cout << it->first << ":" << std::endl;
			std::cout << it->second.transpose() << std::endl;
		}

		std::cout << "DH_alpha_params_map_ size: " << DH_alpha_params_map_.size() << std::endl;
		for (auto it = DH_alpha_params_map_.begin(); it != DH_alpha_params_map_.end(); ++it) {
			std::cout << it->first << ":" << std::endl;
			std::cout << it->second.transpose() << std::endl;
		}
	}

	Eigen::Affine3d Forward::fwd_kin_solve(const std::vector<double> &q_vec, std::string kinematic_set_name,
			const unsigned int desired_joint) {
		std::vector<double> tq = q_vec;
		if (tq.size() == 6) {
			tq.push_back(0.0);
		}
		Vectorq7x1 q;
		q[0] = tq[0];
		q[1] = tq[1];
		q[2] = tq[2];
		q[3] = tq[3];
		q[4] = tq[4];
		q[5] = tq[5];
		q[6] = tq[6];
		return Forward::fwd_kin_solve(q, kinematic_set_name, desired_joint);
	}

	Eigen::Affine3d Forward::fwd_kin_solve(const Vectorq7x1 &q_vec, std::string kinematic_set_name, const unsigned int desired_joint) {
		this->current_joint_state__map_[kinematic_set_name] = q_vec;
		unsigned int joint = desired_joint - 1;

		Eigen::VectorXd thetas_DH_vec, dvals_DH_vec;
		convert_qvec_to_DH_vecs(q_vec, thetas_DH_vec, dvals_DH_vec, kinematic_set_name);
		fwd_kin_solve_DH(thetas_DH_vec, dvals_DH_vec, kinematic_set_name);

		if (joint < 6) {
			return this->affine_products_map_[kinematic_set_name][joint];
		} else {
			return this->affine_gripper_wrt_base_map_[kinematic_set_name];
		}
	}

	void Forward::convert_qvec_to_DH_vecs(const Vectorq7x1 &q_vec, Eigen::VectorXd &thetas_DH_vec, Eigen::VectorXd &dvals_DH_vec,
			std::string kinematic_set_name) {
		thetas_DH_vec.resize(7);
		thetas_DH_vec = this->theta_DH_offsets_map_[kinematic_set_name];

		for (int i = 0; i < 7; i++) {
			if (i == 0) {
				// Joint 1 has a scale factor to be added here.
				thetas_DH_vec(i) += this->j1_scale_factor_map_[kinematic_set_name] * q_vec(i);
			} else if (i == 1) {
				// Joint 2 has a scale factor to be added here.
				thetas_DH_vec(i) += this->j2_scale_factor_map_[kinematic_set_name] * q_vec(i);
			} else if (i == 2) {
				// skip the linear joint.
				continue;
			} else {
				thetas_DH_vec(i) += q_vec(i);
			}
		}

		dvals_DH_vec.resize(7);
		dvals_DH_vec = this->dval_DH_offsets_map_[kinematic_set_name];

		// Prismatic
		dvals_DH_vec(2) = this->j3_scale_factor_map_[kinematic_set_name] * q_vec(2) + dvals_DH_vec(2);

	}

	void Forward::fwd_kin_solve_DH(const Eigen::VectorXd &theta_vec, const Eigen::VectorXd &d_vec, std::string kinematic_set_name) {

		std::vector<Eigen::Affine3d> affines_i_wrt_iminus1;
		affines_i_wrt_iminus1.resize(7);
		Eigen::Affine3d xform;
		double a, d, theta, alpha;

		Eigen::VectorXd DH_a_params, DH_alpha_params;
		std::vector<Eigen::Affine3d> affine_products;
		DH_a_params = this->DH_a_params_map_[kinematic_set_name];
		DH_alpha_params = this->DH_alpha_params_map_[kinematic_set_name];

		for (int i = 0; i < 7; i++) {
			a = DH_a_params[i];
			d = d_vec(i);
			alpha = DH_alpha_params[i];
			theta = theta_vec(i);
			xform = computeAffineOfDH(a, d, alpha, theta);
			affines_i_wrt_iminus1[i] = xform;
		}

		affine_products.resize(7);
		affine_products[0] = this->affine_frame0_wrt_base_ * affines_i_wrt_iminus1[0];
		for (int i = 1; i < 7; i++) {
			affine_products[i] = affine_products[i - 1] * affines_i_wrt_iminus1[i];
		}

		this->affine_gripper_wrt_base_map_[kinematic_set_name] = affine_products[6] * this->affine_gripper_wrt_frame6_;

		// RN added for wrist pt coordinate w/rt base frame
		this->affine_wrist_wrt_base_map_[kinematic_set_name] = affine_products[2];

		// Also include the newly calculated affine products into its map.
		this->affine_products_map_[kinematic_set_name] = affine_products;
	}

	Eigen::MatrixXd Forward::compute_jacobian(const Vectorq7x1 &q_vec, std::string kinematic_set_name) {

		Eigen::MatrixXd temp_jacobian;
		temp_jacobian = Eigen::MatrixXd::Zero(6, 6);

		// use the jacobian to make the computation.
		fwd_kin_solve(q_vec, kinematic_set_name);

		Eigen::Vector3d z_axis;
		Eigen::Vector3d vec_tip_minus_Oi_wrt_base;
		Eigen::Matrix3d R;
		Eigen::Vector3d r_tip_wrt_base = this->affine_gripper_wrt_base_map_[kinematic_set_name].translation();
		Eigen::Vector3d z_axis0 = this->affine_frame0_wrt_base_.linear().col(2);
		std::vector<Eigen::Affine3d> affine_products;

		affine_products = this->affine_products_map_[kinematic_set_name];

		// angular Jacobian is just the z axes of each revolute joint (expressed in base frame);
		// for prismatic joint, there is no angular contribution
		// start from z_axis0

		// Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
		temp_jacobian.block<3, 1>(3, 0) = z_axis0;

		vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - this->affine_frame0_wrt_base_.translation();
		temp_jacobian.block<3, 1>(0, 0) = z_axis0.cross(vec_tip_minus_Oi_wrt_base);
		// 2nd joint:
		// refer to previous joint's z axis

		R = affine_products[0].linear();
		z_axis = R.col(2);
		// Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
		temp_jacobian.block<3, 1>(3, 1) = z_axis;
		vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - affine_products[0].translation();
		temp_jacobian.block<3, 1>(0, 1) = z_axis.cross(vec_tip_minus_Oi_wrt_base);

		// prismatic joint:
		R = affine_products[1].linear();
		z_axis = R.col(2);
		temp_jacobian.block<3, 1>(0, 2) = z_axis;

		// joints 4-6:
		for (int i = 3; i < 6; i++) {
			R = affine_products[i - 1].linear();
			z_axis = R.col(2);
			// Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
			temp_jacobian.block<3, 1>(3, i) = z_axis;
			vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - affine_products[i - 1].translation();
			temp_jacobian.block<3, 1>(0, i) = z_axis.cross(vec_tip_minus_Oi_wrt_base);
		}

		this->Jacobian_map_[kinematic_set_name] = temp_jacobian;

		// translational Jacobian depends on joint's z-axis and vector from i'th axis to robot tip
		return temp_jacobian;
	}
}  // namespace davinci_kinematics
