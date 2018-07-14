//
// Created by william on 13/07/18.
//


#include <ctime>
#include <iostream>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>

#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>




int main(int argc, char **argv)
{

  davinci_kinematics::Forward dvrk_forward;
  davinci_kinematics::Inverse dvrk_inverse;

  Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base;
  davinci_kinematics::Vectorq7x1 q_vec, q_vec_up, q_vec_bottom;
  davinci_kinematics::Vectorq7x1 err_vec, q_vec_ik, q_vec_ik_refined, q_vec_frozen_ik_refined;

  ROS_WARN("Forward test:");

  q_vec(0) = 0;
  q_vec(1) = 0;
  q_vec(2) = 0.15;
  q_vec(3) = 0;
  q_vec(4) = 0;
  q_vec(5) = 0;
  q_vec(6) = 0;

  std::cout << "q_vec#=================================================" << ": " << std::endl;
  std::cout << q_vec.transpose() << std::endl << std::endl;

  affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);

  std::cout << "q_vec#" << ".affine_gripper_wrt_base.translation():" << std::endl;
  std::cout << affine_gripper_wrt_base.translation() << std::endl << std::endl;

  ROS_WARN("Inverse Test:");

  if (dvrk_inverse.ik_solve(affine_gripper_wrt_base) > 0) {

    q_vec_ik = dvrk_inverse.get_soln();
    std::cout << "q_vec_ik#=================================================" << ": " << std::endl;
    std::cout << q_vec_ik.transpose() << std::endl << std::endl;

  }


  return 1;
}