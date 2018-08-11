//
// Created by william on 30/07/18.
//

#include <iostream>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>

#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>


Eigen::Affine3d printInfo(Eigen::Vector3d test_pt) {

  davinci_kinematics::Forward dvrk_forward;
  davinci_kinematics::Inverse dvrk_inverse;

  Eigen::Affine3d affine_wrist_wrt_base, affine_tip_wrt_base;

  davinci_kinematics::Vectorq7x1 q_vec_ik;

  dvrk_inverse.resetDhOffsetsMaps();
  dvrk_inverse.loadDHyamlfiles("psm1_dh","psm1_dh");
    dvrk_inverse.loadDHyamlfiles("psm2_dh","psm2_dh");
//  dvrk_inverse.loadDHyamlfiles("psm1_dh_sim","psm1_dh_sim");
//  dvrk_inverse.loadDHyamlfiles("psm_generic","psm_generic");

  if (dvrk_inverse.ik_solve_frozen_refined(test_pt, "psm2_dh") > 0) {

    ROS_INFO("Got a Frozen IK solution");
    q_vec_ik = dvrk_inverse.get_soln_frozon_ik_refined("psm2_dh");
    std::cout << "q_vec_ik: " << std::endl << q_vec_ik.transpose() << std::endl;

    affine_tip_wrt_base = dvrk_inverse.fwd_kin_solve(q_vec_ik, "psm2_dh");

    affine_wrist_wrt_base = dvrk_inverse.get_wrist_wrt_base("psm2_dh");


    std::cout << "affine_wrist_wrt_base: " << std::endl
              << affine_wrist_wrt_base.translation().transpose() << std::endl;

//    std::cout << "affine_tip_wrt_base: " << std::endl
//              << affine_tip_wrt_base.translation().transpose() << std::endl;

  } else {
    ROS_WARN("Failed to get a solution");
  }

  return affine_wrist_wrt_base;

}


Eigen::Affine3d printInfo(davinci_kinematics::Vectorq7x1 q_vec) {


  davinci_kinematics::Inverse dvrk_inverse;

  Eigen::Affine3d affine_wrist_wrt_base, affine_tip_wrt_base;

//  davinci_kinematics::Vectorq7x1 q_vec_ik;

  dvrk_inverse.resetDhOffsetsMaps();
  dvrk_inverse.loadDHyamlfiles("psm1_dh","psm1_dh");
  dvrk_inverse.loadDHyamlfiles("psm2_dh","psm2_dh");
//  dvrk_inverse.loadDHyamlfiles("psm1_dh_sim","psm1_dh_sim");
//  dvrk_inverse.loadDHyamlfiles("psm_generic","psm_generic");

    std::cout << "Your input q_vec: " << std::endl << q_vec.transpose() << std::endl;

    affine_tip_wrt_base = dvrk_inverse.fwd_kin_solve(q_vec, "psm2_dh");

    affine_wrist_wrt_base = dvrk_inverse.get_wrist_wrt_base("psm2_dh");


    std::cout << "affine_wrist_wrt_base: " << std::endl
              << affine_wrist_wrt_base.translation().transpose() << std::endl;


  return affine_wrist_wrt_base;

}



int main(int argc, char **argv)
{
  Eigen::Vector3d test_pt;
  std::vector<Eigen::Affine3d> wrist_affines;

  davinci_kinematics::Vectorq7x1 q_vec;

  wrist_affines.clear();

//  test_pt << 0.03,0.02,-0.16;
//  wrist_affines.push_back(printInfo(test_pt));
//  test_pt << -0.01,0.02,-0.16;
//  wrist_affines.push_back(printInfo(test_pt));
//
//  test_pt << 0.03,0.06,-0.16;
//  wrist_affines.push_back(printInfo(test_pt));
//  test_pt << -0.01,0.06, -0.16;
//  wrist_affines.push_back(printInfo(test_pt));
//
//  test_pt << 0.03,0.02,-0.20;
//  wrist_affines.push_back(printInfo(test_pt));
//  test_pt << -0.01,0.02,-0.20;
//  wrist_affines.push_back(printInfo(test_pt));
//
//  test_pt << 0.03,0.06,-0.20;
//  wrist_affines.push_back(printInfo(test_pt));
//  test_pt << -0.01,0.06, -0.20;
//  wrist_affines.push_back(printInfo(test_pt));



  q_vec << 0.177, -0.132, 0.160, 0, 0, 0, 0;
  wrist_affines.push_back(printInfo(q_vec));

  q_vec << -0.070, -0.133, 0.158, 0, 0, 0, 0;
  wrist_affines.push_back(printInfo(q_vec));

  q_vec << 0.179, -0.355, 0.170, 0, 0, 0, 0;
  wrist_affines.push_back(printInfo(q_vec));

  q_vec << -0.068, -0.360, 0.167, 0, 0, 0, 0;
  wrist_affines.push_back(printInfo(q_vec));

  q_vec << 0.144, -0.108, 0.200, 0, 0, 0, 0;
  wrist_affines.push_back(printInfo(q_vec));

  q_vec << -0.054, -0.109, 0.198, 0, 0, 0, 0;
  wrist_affines.push_back(printInfo(q_vec));

  q_vec << 0.146, -0.293, 0.208, 0, 0, 0, 0;
  wrist_affines.push_back(printInfo(q_vec));

  q_vec << -0.052, -0.295, 0.206, 0, 0, 0, 0;
  wrist_affines.push_back(printInfo(q_vec));

  std::cout << "% ---" << std::endl;
  std::cout << "wrist_pt_1 = [" << wrist_affines[0].translation().transpose()  << " 1];" << std::endl;
  std::cout << "wrist_pt_2 = [" << wrist_affines[1].translation().transpose()  << " 1];" << std::endl;
  std::cout << "wrist_pt_3 = [" << wrist_affines[2].translation().transpose()  << " 1];" << std::endl;
  std::cout << "wrist_pt_4 = [" << wrist_affines[3].translation().transpose()  << " 1];" << std::endl;
  std::cout << "wrist_pt_5 = [" << wrist_affines[4].translation().transpose()  << " 1];" << std::endl;
  std::cout << "wrist_pt_6 = [" << wrist_affines[5].translation().transpose()  << " 1];" << std::endl;
  std::cout << "wrist_pt_7 = [" << wrist_affines[6].translation().transpose()  << " 1];" << std::endl;
  std::cout << "wrist_pt_8 = [" << wrist_affines[7].translation().transpose()  << " 1];" << std::endl;
  std::cout << "% ---" << std::endl;


  return 1;
}
