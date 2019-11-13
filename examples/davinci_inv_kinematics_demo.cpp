// @TODO Add License Text.
// Copyright Wyatt S. Newman 2015 and Russell Jackson 2017

#include <iostream>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>

#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>

// send out random legal joint values; test resulting single solution
int main(int argc, char **argv)
{
  davinci_kinematics::Forward dvrk_forward;
  davinci_kinematics::Inverse dvrk_inverse;

  Eigen::Vector3d w_wrt_base, q123;

  davinci_kinematics::Vectorq7x1 q_vec, err_vec, q_vec_ik;
  q_vec << 0, 0, 0, 0, 0, 0, 0;
  int err_cnt = 0;

  Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base, affine_frame_wrt_base;
  // wait to start receiving valid tf transforms

  Eigen::Vector3d tip_from_FK, tip_from_FK_of_IK, tip_err;
  // note: with print-outs, takes about 45sec for 10,000 iterations, and got 0 errors
  for (int itries = 0; itries < 50; itries++)
  {
    dvrk_forward.gen_rand_legal_jnt_vals(q_vec);

    std::cout << "using q_vec = " << q_vec.transpose() << std::endl;
    printf("gripper tip frame from FK: \n");
    affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
    std::cout << "affine linear (R): " << std::endl;
    std::cout << affine_gripper_wrt_base.linear() << std::endl;
    tip_from_FK = affine_gripper_wrt_base.translation();
    std::cout << "origin: " << tip_from_FK.transpose() << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;

    if (dvrk_inverse.ik_solve(affine_gripper_wrt_base) > 0)
    {
      // Validate the legal solution.
      q_vec_ik = dvrk_inverse.get_soln();

      printf("FK of IK soln: \n");
      affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec_ik);
      std::cout << "affine linear (R): " << std::endl;
      std::cout << affine_gripper_wrt_base.linear() << std::endl;
      tip_from_FK_of_IK = affine_gripper_wrt_base.translation();
      std::cout << "origin: ";
      std::cout << tip_from_FK_of_IK.transpose() << std::endl;

      std::cout << std::endl;
      std::cout << "q_vec in: " << q_vec.transpose() << std::endl;
      std::cout << "q_vec_ik: " << q_vec_ik.transpose() << std::endl;
      err_vec = q_vec - q_vec_ik;

      std::cout << "err vec: " << err_vec.transpose() << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      tip_err = tip_from_FK - tip_from_FK_of_IK;
      std::cout << "tip err: " << tip_err.transpose() << std::endl;
      printf("jspace errvec norm: %f\n", err_vec.norm());
      printf("tip pos err norm: %f:\n", tip_err.norm());
      if (err_vec.norm() + tip_err.norm() > 0.0001)
      {
        printf("excessive error!");
        err_cnt++;
      }
      printf("itries = %d; err_cnt = %d", itries, err_cnt);
    }
  }
  printf("err_cnt = %d", err_cnt);

  std::cout << "hello";
  return 0;
}
