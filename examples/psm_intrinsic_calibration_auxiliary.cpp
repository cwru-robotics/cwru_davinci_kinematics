//
// Created by william on 30/07/18.
//

#include <iostream>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>

#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>


void printInfo(Eigen::Vector3d test_pt) {

  davinci_kinematics::Forward dvrk_forward;
  davinci_kinematics::Inverse dvrk_inverse;

  Eigen::Affine3d affine_wrist_wrt_base, affine_tip_wrt_base;

  davinci_kinematics::Vectorq7x1 q_vec_ik;

  dvrk_inverse.resetDhOffsetsMaps();
  dvrk_inverse.loadDHyamlfiles("psm1_dh","psm1_dh");
  dvrk_inverse.loadDHyamlfiles("psm1_dh","psm1_dh_sim");
  dvrk_inverse.loadDHyamlfiles("psm_generic","psm_generic");

  if (dvrk_inverse.ik_solve_frozen_refined(test_pt, "psm1_dh_sim") > 0) {

    ROS_INFO("Got a Frozen IK solution");
    q_vec_ik = dvrk_inverse.get_soln_frozon_ik_refined("psm1_dh_sim");
    std::cout << "q_vec_ik: " << std::endl << q_vec_ik.transpose() << std::endl;

    affine_tip_wrt_base = dvrk_inverse.fwd_kin_solve(q_vec_ik, "psm1_dh_sim");

    affine_wrist_wrt_base = dvrk_inverse.get_wrist_wrt_base("psm1_dh_sim");


    std::cout << "affine_wrist_wrt_base: " << std::endl
              << affine_wrist_wrt_base.translation().transpose() << std::endl;

//    std::cout << "affine_tip_wrt_base: " << std::endl
//              << affine_tip_wrt_base.translation().transpose() << std::endl;

  } else {
    ROS_WARN("Failed to get a solution");
  }

}

int main(int argc, char **argv)
{
  Eigen::Vector3d test_pt;
  test_pt << 0.03,0.02,-0.16;
  printInfo(test_pt);
  test_pt << -0.01,0.02,-0.16;
  printInfo(test_pt);

  test_pt << 0.03,0.06,-0.16;
  printInfo(test_pt);
  test_pt << -0.01,0.06, -0.16;
  printInfo(test_pt);

  test_pt << 0.03,0.02,-0.20;
  printInfo(test_pt);
  test_pt << -0.01,0.02,-0.20;
  printInfo(test_pt);

  test_pt << 0.03,0.06,-0.20;
  printInfo(test_pt);
  test_pt << -0.01,0.06, -0.20;
  printInfo(test_pt);



  return 1;
}