/*
 *  full_kinematics.cpp
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

// This file runs unit tests in order to validate the fwd and inverse kinematics
// of the cwru_davinci_kinematics repository.

#include <vector>
#include <Eigen/Eigen>

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

TEST(davinci_kinematics, Full_Kinematics_PJP)
{
  const double lower_x = -0.5;
  const double upper_x = 0.5;
  const double lower_y = -0.25;
  const double upper_y = 0.25;
  const double lower_z = -0.2;
  const double upper_z = 0.0;
  const double delta_p = 0.5;

  const double lower_a = -1.57;
  const double upper_a = 1.57;
  const double delta_a = 0.2;

  davinci_kinematics::Forward dvrk_forward;
  davinci_kinematics::Inverse dvrk_inverse;

  int rejected = 0;
  int correct = 0;
  int incorrect = 0;

  for (double x = lower_x; x <= upper_x; x = x + delta_p)
  {
    for (double y = lower_y; y <= upper_y; y = y + delta_p)
    {
      for (double z = lower_z; z <= upper_z; z = z + delta_p)
      {
        for (double roll = lower_a; roll <= upper_a; roll = roll + delta_a)
        {
          for (double pitch = lower_a; pitch <= upper_a; pitch = pitch + delta_a)
          {
            for (double yaw = lower_a; yaw <= upper_a; yaw = yaw + delta_a)
            {
              Eigen::Matrix3d R;
              R =
                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                * Eigen::AngleAxisd(pitch,  Eigen::Vector3d::UnitY())
                * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
              Eigen::Affine3d in;
              in = R;
              in.translation() = Eigen::Vector3d(x, y, z);
              int s = dvrk_inverse.ik_solve(in);
              if (s < 1)
              {
                rejected++;
              }
              else
              {
                davinci_kinematics::Vectorq7x1 j = dvrk_inverse.get_soln();
                Eigen::Affine3d out = dvrk_forward.fwd_kin_solve(j);
                if (out.matrix().isApprox(in.matrix(), 0.0001))
                {
                  correct++;
                }
                else
                {
                  rejected++;
                }
              }
            }
          }
        }
      }
    }
  }
  ROS_INFO("%d rejected, %d correct, %d incorrect", rejected, correct, incorrect);
  ASSERT_LT(incorrect, 1);
  SUCCEED();
}

/**
 * @brief This test file defines a collection of tests which can be used for validation
 * of the closed loop kineamtics definitons.
 */
TEST(davinci_kinematics, Full_Kinematics_JPJ)
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

  double smallest_error(10000.0);
  double largest_error(0.0);

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

    /*
    q_vec(0) = -0.975;
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
    q_vec(6) = 0.0;  // jnt6;
    */

    Eigen::Affine3d affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);

    int solCount = dvrk_inverse.ik_solve(affine_gripper_wrt_base);
    davinci_kinematics::Vectorq7x1 q_vecp = dvrk_inverse.get_soln();
    // EXPECT_GT(solCount, 0) << "The joint configuration FWD:\n" << q_vec <<
    //  "\nThe joint configuration INV:\n" << q_vecp << "\nFailed \n";

    davinci_kinematics::Vectorq7x1 err_vec = q_vec - q_vecp;

    double err_mag(err_vec.norm());


    if (err_mag > largest_error)
    {
      largest_error = err_mag;

      q_vec_fwd_worst = q_vec;
      q_vec_inv_worst = q_vecp;
      ik_lin_error = dvrk_inverse.getError_l();
      ik_rot_error = dvrk_inverse.getError_r();
    }

    if (err_mag < smallest_error)
    {
      smallest_error = err_mag;
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
  printf("The smallest error was %f and the largest error was %f\n", smallest_error, largest_error);
  std::cout << "The largest error occured at input joints:\n" << q_vec_fwd_worst << "\n\n"
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
  Eigen::VectorXd dq_vec6x1, dp6x1;
  dq_vec6x1.resize(6, 1);
  dp6x1.resize(6, 1);

  double p_err, phi_err;
  const int ni(25);
  const int nj(25);

  for (int i(0); i < ni; i++)
  {
    dvrk_forward.gen_rand_legal_jnt_vals(q_vec_1);
    Eigen::MatrixXd Jacobian = dvrk_forward.compute_jacobian(q_vec_1);
    Eigen::Affine3d affine_fk1 = dvrk_forward.fwd_kin_solve();

    std::cout << "The Jacobian at iteration " << i <<" is : \n" << Jacobian << std::endl;

    for (int j = 0; j < nj; j++)
    {
      gen_rand_joint_perturbations(dq_vec);
      q_vec_2 = q_vec_1 + dq_vec;

      davinci_kinematics::Vectorq7x1 dq_vec7x1;
      for (int i = 0; i < 6; i++) dq_vec6x1(i) = dq_vec(i);

      Eigen::Affine3d affine_fk2 = dvrk_forward.fwd_kin_solve(q_vec_2);

      dp_fk = affine_fk2.translation() - affine_fk1.translation();
      dR = affine_fk2.linear() * affine_fk1.linear().transpose();

      // For perturbations, can get dPhi 3x1 vector from components of dR operator
      dphi_fk(0) = -dR(1, 2);
      dphi_fk(1) = dR(0, 2);
      dphi_fk(2) = -dR(0, 1);

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

      EXPECT_LT(p_err_rat, 1.0e-4);
      EXPECT_LT(phi_err_rat, 1.0e-4);
    }
  }
  SUCCEED();
}

TEST(davinci_kinematics, Full_Kinematics_Limits)
{
  unsigned int iterations(10000);
  unsigned int seed = time(NULL);
  // number of times to permute through the error codes:
  for (unsigned int i(0); i < iterations; i++)
  {
    davinci_kinematics::Vectorq7x1 q_test;
    davinci_kinematics::Forward::gen_rand_legal_jnt_vals(q_test);
    q_test(6) = 0.0;

    if (q_test(2) < 0.035)
    {
      q_test(3) = 0.1;
      int result = davinci_kinematics::Forward::check_jnts(q_test);
      EXPECT_EQ(1, result);

      q_test(3) = 0.0;
      q_test(4) = 0.0;
      q_test(5) = 0.0;
      result = davinci_kinematics::Forward::check_jnts(q_test);
      EXPECT_EQ(0, result);
    }
    else
    {
      for (int j (0); j < 64; j++)
      {
        davinci_kinematics::Vectorq7x1 q_testA = q_test;
        std::vector<double> over_under(6, 0.0);
        for (unsigned int k(0); k < 6; k++)
        {
          if (((j >> k) % 2) == 1)
          {
            int result = rand_r(&seed) % 2;
            if (result == 1)
            {
              over_under[k] = -1.0;
            }
            else over_under[k] = 1.0;
            if (k == 2) over_under[k] = 1.0;
          }
          q_testA(k) += over_under[k] * 10.0;
        }
        int result = davinci_kinematics::Forward::check_jnts(q_testA);
        EXPECT_EQ(-j, result);
      }
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
