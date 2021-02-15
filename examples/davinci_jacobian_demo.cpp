// @TODO Add License Text.
// Copyright Wyatt S. Newman 2015 and Russell Jackson 2017
// davinci_Jacobian_test_main.cpp
// get random legal joint values, q_vec; compute fk1
// generate random perturbations, dq; compute fk2
// compute dp_fk = fk2_translational()-fk1_translational()
// compute Jacobian
// compute dp_J = J*dq
// compare dp_J to dp_fk
// do analogous comparison for dphi1, dphi2, dphi3

// key lines of this program (do the following to use this Jacobian):
// Davinci_fwd_solver davinci_fwd_solver; //instantiate a forward-kinematics solver
// Jacobian = davinci_fwd_solver.compute_jacobian(q_vec1); //compute the Jacobian
// dp6x1 = Jacobian*dq_vec6x1; // here is the Jacobian-based approximation of incremental Cartesian pose change

#include <davinci_kinematics/davinci_fwd_kinematics.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry>

// fnc to generate random joint perturbations in range set by eps
void gen_rand_joint_perturbations(davinci_kinematics::Vectorq7x1 &dqvec, double eps = 0.000001) {
	static unsigned int seed = time(NULL);
	dqvec(6) = 0;
	for (int i = 0; i < 6; i++) {
		dqvec(i) = eps * (1.0 - 0.5 * static_cast<double>(rand_r(&seed)) / static_cast<double>(RAND_MAX));
	}
}

/**
 * @brief This program demonstrates the numerical jacobian
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "davinci_Jacobian_test_main");

	davinci_kinematics::Forward davinci_fwd_solver;
	Eigen::MatrixXd Jacobian;

	Eigen::MatrixXd rn_test;

	Eigen::Vector3d dp_fk, dp_J, dp_err_vec;
	Eigen::Matrix3d R1, R2, dR;
	Eigen::Vector3d dphi_J, dphi_fk, dphi_err_vec;
	double p_err, phi_err;
	int ans;

	davinci_kinematics::Vectorq7x1 q_vec1, q_vec2, dq_vec;
	Eigen::VectorXd dq_vec6x1, dp6x1;
	for (int i = 0; i < 7; i++)
		q_vec1(i) = 0.0;

	q_vec2 = q_vec1;
	dq_vec = q_vec1;

	// 6x1 perturbation vector to be used with Jacobian
	dq_vec6x1.resize(6, 1);
	dp6x1.resize(6, 1);
	// holders for FK computations
	Eigen::Affine3d affine_fk1, affine_fk2;

	for (int itries = 0; itries < 50; itries++) // used to be 10000
			{
		davinci_fwd_solver.gen_rand_legal_jnt_vals(q_vec1);
		std::cout << "using q_vec = " << q_vec1.transpose() << std::endl;
		gen_rand_joint_perturbations(dq_vec);
		q_vec2 = q_vec1 + dq_vec;
		// strip off 7'th joint variable, gripper opening angle, to get 6x1 vector of joint perturbations
		for (int i = 0; i < 6; i++)
			dq_vec6x1(i) = dq_vec(i);

		// compute FK for two joint-space poses, q_vec1, q_vec2, that are
		// very close to each other (random perturbations)
		// pose from first q_vec
		ROS_INFO("gripper tip frame from FK: ");
		affine_fk1 = davinci_fwd_solver.fwd_kin_solve(q_vec1);
		affine_fk2 = davinci_fwd_solver.fwd_kin_solve(q_vec2);
		// orientation from first q_vec
		std::cout << "affine linear (R): " << std::endl;
		std::cout << affine_fk1.linear() << std::endl << std::endl;
		std::cout << "origin: " << affine_fk1.translation().transpose() << std::endl << std::endl;
		// compute numerical perturbation of output, based on two FK calls:
		dp_fk = affine_fk2.translation() - affine_fk1.translation();
		std::cout << "tip displacement due to perturbation of angles, per FK:" << std::endl;
		std::cout << "dp_fk = " << dp_fk.transpose() << std::endl << std::endl;
		// What rotation operator is required to change from orientation 1 to orientation 2?
		dR = affine_fk2.linear() * affine_fk1.linear().transpose();
		std::cout << "Rotation operator for perturbation, per FK: " << std::endl;
		std::cout << dR << std::endl;
		// For perturbations, can get dPhi 3x1 vector from components of dR operator
		dphi_fk(0) = -dR(1, 2);
		dphi_fk(1) = dR(0, 2);
		dphi_fk(2) = -dR(0, 1);
		std::cout << "extracted dPhi vector from rotation operator:" << std::endl;
		std::cout << "dphi_fk = " << dphi_fk.transpose() << std::endl;

		// here's a call to the function under test: computing the Jacobian about q_vec1
		// compute the Jacobian
		Jacobian = davinci_fwd_solver.compute_jacobian(q_vec1);
		std::cout << "computed Jacobian: " << std::endl;
		std::cout << Jacobian << std::endl << std::endl;
		std::cout << "perturbation of first 6 joints: ";
		std::cout << "dq_vec6x1 = " << dq_vec6x1.transpose() << std::endl;

		// here is the Jacobian-based approximation of incremental Cartesian pose change
		dp6x1 = Jacobian * dq_vec6x1;
		// first three elements are dx,dy,dz, and next 3 are dphix, dphiy, dphiz
		std::cout << "6x1 perturbation of pose per Jacobian and joint perturbations:" << std::endl;
		std::cout << "dp6x1 = " << dp6x1.transpose() << std::endl;
		// strip off dx, dy, dz
		dp_J = dp6x1.block<3, 1>(0, 0);
		// compare Jacobian dp(dq) to FK dp(dq)
		dp_err_vec = dp_fk - dp_J;
		std::cout << "comparison of endpoint displacements, FK vs Jacobian: " << std::endl;
		std::cout << "dp_err_vec = " << dp_err_vec.transpose() << std::endl;
		p_err = dp_err_vec.norm();
		std::cout << "dp err norm = " << p_err << std::endl;
		// express this as displacement disagreement vs magnitude of displacement
		// e.g., percentage err (but expressed as a fraction, not percentage)
		std::cout << "fractional error: " << p_err / (dp_fk.norm()) << std::endl << std::endl;

		// repeat for eval of angular Jacobian:
		// (dphi_x, dphi_y, dphi_z)
		dphi_J = dp6x1.block<3, 1>(3, 0);
		dphi_err_vec = dphi_fk - dphi_J;
		std::cout << "comparison of incremental rotation vectors, FK vs Jacobian: " << std::endl;
		std::cout << "dphi_err_vec = " << dphi_err_vec.transpose() << std::endl;
		phi_err = dphi_err_vec.norm();
		std::cout << "dphi err norm = " << phi_err << std::endl;
		// express as a normalized error (scalar)
		std::cout << "fractional error: " << phi_err / (dphi_fk.norm()) << std::endl << std::endl;

		std::cout << "The inverse of B is:\n" << Jacobian.inverse() << std::endl;
		rn_test = Jacobian * Jacobian.inverse();
		std::cout << "The B*inv(B):\n" << rn_test << std::endl;

	}
}
