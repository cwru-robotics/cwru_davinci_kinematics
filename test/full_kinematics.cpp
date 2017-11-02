// @TODO Add License Text.
// Copyright Wyatt S. Newman 2015 and Russell Jackson 2017


#include <sensor_msgs/JointState.h>
#include <gtest/gtest.h>

#include <cwru_davinci_kinematics/davinci_fwd_kinematics.h>
#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>

using davinci_kinematics::q_upper_limits;
using davinci_kinematics::q_lower_limits;

// fnc to generate random joint perturbations in range set by eps
void gen_rand_joint_perturbations(davinci_kinematics::Vectorq7x1 &dqvec, double eps = 0.000001)
{
  static unsigned int seed = time(NULL);
  dqvec(6) = 0;
  for (int i = 0; i < 6; i++)
  {
    dqvec(i) = eps * (1.0 - 0.5*static_cast<double> (rand_r(&seed)) / static_cast<double> (RAND_MAX));
  }
}

/**
 * @brief This test file defines a collection of tests which can be used for validation
 * of the closed loop kineamtics definitons.
 */
TEST(davinci_kinematics, Full_Kinematics_KI)
{
  // randomly generate a joint position:
  // compute the corresponding FK,
  // Compute the IK
  // compare the new joints with the original joints.

  davinci_kinematics::Vectorq7x1  q_vec;
  davinci_kinematics::Forward dvrk_forward;
  davinci_kinematics::Inverse dvrk_inverse;

  double joint_steps(4);
  double dj[7];
  for (int index(0); index < 7; index++)
  {
    dj[index] = (q_upper_limits[index] - q_lower_limits[index]) / joint_steps;
    if (index != 2)
    {
      dj[index] *= 1.5;
    }
  }

  uint64_t good_count(0);
  uint64_t bad_count(0);
  uint64_t total_count(0);

  double best_error(10000.0);

  double worst_error(0.5);
  double ik_lin_error(0.0);
  double ik_rot_error(0.0);
  davinci_kinematics::Vectorq7x1  q_vec_fwd_worst, q_vec_inv_worst;
  q_vec_fwd_worst = davinci_kinematics::Vectorq7x1::Zero();
  q_vec_inv_worst = davinci_kinematics::Vectorq7x1::Zero();


  // dense sequential analysis.
  // TODO(wsn) Currently the joint limits have to be padded in order to keep the solution boundaries stable.
  for (double jnt0 = q_lower_limits[0] + 0.0001; jnt0 < q_upper_limits[0] - 0.0001; jnt0 += dj[0])
    for (double jnt1 = q_lower_limits[1] + 0.0001; jnt1 < q_upper_limits[1] - 0.0001; jnt1 += dj[1])
      for (double jnt2 = 0.0157; jnt2 < q_upper_limits[2] - 0.0001; jnt2 += dj[2])  // This joint is sensitive?
        for (double jnt3 = q_lower_limits[3] + 0.0001; jnt3 < q_upper_limits[3] - 0.0001; jnt3 += dj[3])
          for (double jnt4 = q_lower_limits[4] + 0.0001; jnt4 < q_upper_limits[4] - 0.0001; jnt4 += dj[4])
            for (double jnt5 = q_lower_limits[5] + 0.0001; jnt5 < q_upper_limits[5] - 0.0001; jnt5 += dj[5])
              // for (double jnt6 = q_lower_limits[6]; jnt6 < q_upper_limits[6]; jnt6 += dj[6])
  {
    q_vec(0) = jnt0;
    q_vec(1) = jnt1;
    q_vec(2) = jnt2;
    q_vec(3) = jnt3;
    q_vec(4) = jnt4;
    q_vec(5) = jnt5;
    q_vec(6) = 0.0;  // jnt6;

    /* q_vec(0) = -0.975;
    q_vec(1) = -0.6825;
    q_vec(2) = 0.0375;
    q_vec(3) = -2.19375;
    q_vec(4) = 0.8321;
    q_vec(5) = 0.7367;
    q_vec(6) = 0.0;  // jnt6;

    q_vec(0) = -0.975;
    q_vec(1) = -0.6825;
    q_vec(2) = 0.0375;
    q_vec(3) = -1.06875;
    q_vec(4) = -1.5229;
    q_vec(5) = -1.3483;
    q_vec(6) = 0.0;  // jnt6; */


    Eigen::Affine3d affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);

    int solCount = dvrk_inverse.ik_solve(affine_gripper_wrt_base);
    davinci_kinematics::Vectorq7x1 q_vecp = dvrk_inverse.get_soln();
    // EXPECT_GT(solCount, 0) << "The joint configuration FWD:\n" << q_vec <<
    //  "\nThe joint configuration INV:\n" << q_vecp << "\nFailed \n";


    davinci_kinematics::Vectorq7x1 err_vec = q_vec - q_vecp;

    double err_mag(err_vec.norm());

    if (err_mag > worst_error)
    {
      worst_error = err_mag;
      q_vec_fwd_worst = q_vec;
      q_vec_inv_worst = q_vecp;
      ik_lin_error = dvrk_inverse.getError_l();
      ik_rot_error = dvrk_inverse.getError_r();
    }
    if (err_mag < best_error)
    {
      best_error = err_mag;
    }

    if (err_mag > 1.0e-4)
    {
      bad_count++;
    }
    else
    {
      good_count++;
    }
    total_count++;

    // EXPECT_LT(err_mag, 1.0e-4) << "The joint configuration:" << q_vec << "\nFailed \n";
  }
  printf("after iterating through the entire permutation of joints (%ld): %ld were good and %ld were bad\n",
    total_count, good_count, bad_count);
  printf("The best error was %f and the worst error was %f\n", best_error, worst_error);
  std::cout << "The worst error occured at input joints:\n" << q_vec_fwd_worst << "\n\n"
    << q_vec_inv_worst << "\n\n";
  printf("The linear error was %f\n", ik_lin_error);
  printf("The rotational error was %f\n", ik_rot_error);
  // fully random
  for (int i(0); i < 25; i++)
  {
    dvrk_forward.gen_rand_legal_jnt_vals(q_vec);

    Eigen::Affine3d affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);

    int solCount = dvrk_inverse.ik_solve(affine_gripper_wrt_base);
    EXPECT_GT(solCount, 0);

    davinci_kinematics::Vectorq7x1 q_vecp = dvrk_inverse.get_soln();

    davinci_kinematics::Vectorq7x1 err_vec = q_vec - q_vecp;

    double err_mag(err_vec.norm());

    ASSERT_LT(err_mag, 1.0e-4);
  }
  SUCCEED();
}

TEST(davinci_kinematics, Full_Kinematics_Jacobian)
{
  // randomly generate a joint position:
  // compute the corresponding FK,
  // Compute the IK
  // compare the new joints with the original joints.

  davinci_kinematics::Vectorq7x1  q_vec_1;
  davinci_kinematics::Vectorq7x1  dq_vec;
  davinci_kinematics::Vectorq7x1  q_vec_2;
  davinci_kinematics::Forward dvrk_forward;

  Eigen::Vector3d dp_fk, dp_J, dp_err_vec;
  Eigen::Matrix3d R1, R2, dR;
  Eigen::Vector3d dphi_J, dphi_fk, dphi_err_vec;
  Eigen::MatrixXd Jacobian;
  Eigen::VectorXd dq_vec6x1, dp6x1;
  dq_vec6x1.resize(6, 1);
  dp6x1.resize(6, 1);


  double p_err, phi_err;
  const int ni(25);
  const int nj(25);

  for (int i(0); i < ni; i++)
  {
    dvrk_forward.gen_rand_legal_jnt_vals(q_vec_1);

    for (int j = 0; j < nj; j++)
    {
      gen_rand_joint_perturbations(dq_vec);
      q_vec_2 = q_vec_1 + dq_vec;

      davinci_kinematics::Vectorq6x1 dq_vec6x1;
      for (int i = 0; i < 6; i++) dq_vec6x1(i) = dq_vec(i);

      Eigen::Affine3d affine_fk1 = dvrk_forward.fwd_kin_solve(q_vec_1);
      Eigen::Affine3d affine_fk2 = dvrk_forward.fwd_kin_solve(q_vec_2);

      dp_fk = affine_fk2.translation() - affine_fk1.translation();
      dR = affine_fk2.linear()*affine_fk1.linear().transpose();

      // For perturbations, can get dPhi 3x1 vector from components of dR operator
      dphi_fk(0) = -dR(1, 2);
      dphi_fk(1) = dR(0, 2);
      dphi_fk(2) = -dR(0, 1);
      Jacobian = dvrk_forward.compute_jacobian(q_vec_1);

      // here is the Jacobian-based approximation of incremental Cartesian pose change
      dp6x1 = Jacobian * dq_vec6x1;
      // strip off dx, dy, dz
      dp_J = dp6x1.block<3, 1>(0, 0);
      // compare Jacobian dp(dq) to FK dp(dq)
      dp_err_vec = dp_fk - dp_J;

      p_err = dp_err_vec.norm();



      // repeat for eval of angular Jacobian:
      // (dphi_x, dphi_y, dphi_z)
      dphi_J = dp6x1.block<3, 1>(3, 0);
      dphi_err_vec =  dphi_fk - dphi_J;
      phi_err = dphi_err_vec.norm();

      double p_err_rat = p_err/(dp_fk.norm());
      double phi_err_rat = phi_err/(dphi_fk.norm());

      ASSERT_LT(p_err_rat, 1.0e-4);
      ASSERT_LT(phi_err_rat, 1.0e-4);
    }
  }
  SUCCEED();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
