// RN
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

  Eigen::Vector3d w_wrt_base, q123;

  Eigen::Vector3d tip_coordinate;

  davinci_kinematics::Vectorq7x1 q_vec, err_vec, q_vec_ik, q_vec_ik_refined, q_vec_frozen_ik_refined;
  davinci_kinematics::Vectorq7x1 err_vec_refined_ik;
  q_vec << 0, 0, 0, 0, 0, 0, 0;
  int err_cnt = 0;

  int err_cnt_refined = 0;
  int refined_total_count = 0;
  int entry_count = 0;

  Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base, affine_frame_wrt_base, affine_gripper_wrt_base1;
  Eigen::Affine3d affine_wrist_wrt_base2, affine_gripper_wrt_base2, affine_frame_wrt_base2;
  Eigen::Affine3d affine_gripper_wrt_base_frozen;
  // wait to start receiving valid tf transforms

  Eigen::Vector3d tip_from_FK, tip_from_FK_of_IK, tip_from_FK_of_refined_IK, tip_err, tip_err_refined;
  // note: with print-outs, takes about 45sec for 10,000 iterations, and got 0 errors
  for (int itries = 0; itries < 1; itries++)
  {
    dvrk_forward.gen_rand_legal_jnt_vals(q_vec);

    // std::cout << "using q_vec = " << q_vec.transpose() << std::endl;

    // printf("gripper tip frame from FK: \n");
    q_vec << -0.168323, 0.0463498,  0.153502,  0.951687, -0.355543,  -1.25153,    0;

    std::cout << "HELLO q_vec: " << q_vec.transpose() << std::endl;

    affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
    // std::cout << "affine linear (R): " << std::endl;
    // std::cout << affine_gripper_wrt_base.linear() << std::endl;
    tip_from_FK = affine_gripper_wrt_base.translation();
     std::cout << "origin: " << tip_from_FK.transpose() << std::endl;
     std::cout << std::endl;
     std::cout << std::endl;

    if (dvrk_inverse.ik_solve(affine_gripper_wrt_base) > 0)
    {
			// std::cout << "INTO IF.. "<< std::endl;
      // Validate the legal solution.
      entry_count++;
      q_vec_ik = dvrk_inverse.get_soln();

      // printf("FK of IK soln: \n");
      affine_gripper_wrt_base1 = dvrk_forward.fwd_kin_solve(q_vec_ik);
      tip_from_FK_of_IK = affine_gripper_wrt_base1.translation();
      err_vec = q_vec - q_vec_ik;

      // std::cout << std::endl;
      // std::cout << std::endl;
      tip_err = tip_from_FK - tip_from_FK_of_IK;

      if (tip_err.norm() > 0.001)
      {
        printf("excessive error!");
        err_cnt++;
      }

      int refinement_outcome = 9;

      refinement_outcome = dvrk_inverse.ik_solve_refined(affine_gripper_wrt_base);


      if (refinement_outcome > 0)
      {
        refined_total_count ++;

        q_vec_ik_refined = dvrk_inverse.get_soln_refined();
        affine_gripper_wrt_base2 = dvrk_forward.fwd_kin_solve(q_vec_ik_refined);

        tip_from_FK_of_refined_IK = affine_gripper_wrt_base2.translation();

        // std::cout << "q_vec in:            " << q_vec.transpose() << std::endl;
        // std::cout << "q_vec_ik:            " << q_vec_ik.transpose() << std::endl;
        // std::cout << "q_vec_ik_refined:    " << q_vec_ik_refined.transpose() << std::endl;


        err_vec_refined_ik = q_vec - q_vec_ik_refined;
        // std::cout << "err vec:             " << err_vec.transpose() << std::endl;
        // std::cout << "err_vec_refined_ik:  " << err_vec_refined_ik.transpose() << std::endl;
        // std::cout << std::endl;
        tip_err_refined = tip_from_FK - tip_from_FK_of_refined_IK;
        // std::cout << "tip err:             " << tip_err.transpose() << std::endl;
        // std::cout << "tip_err_refined err: " << tip_err_refined.transpose() << std::endl;
        // std::cout << std::endl;

        // printf("jspace errvec norm: %f\n", err_vec.norm());
        // printf("tip pos err norm: %f:\n", tip_err.norm());
        // printf("jspace errvec norm refined: %f\n", err_vec_refined_ik.norm());
        // printf("tip pos err norm refined: %f:\n", tip_err_refined.norm());
        // std::cout << std::endl;

        // if (err_vec_refined_ik.norm() + tip_err_refined.norm() > 0.001)
        if (tip_err_refined.norm() > 0.001)
        {
          printf("excessive error after refinement!");
          err_cnt_refined++;
        }


        // printf("itries = %d; err_cnt = %d", itries, err_cnt);
        // std::cout << std::endl << "refined_total_count: " << refined_total_count << std::endl
        //   << "entry_count: " << entry_count << std::endl;
        // std::cout << "err_cnt_refined: " << err_cnt_refined << std::endl;

      } else {std::cout << "NO BETTER SOLUTION!" << std::endl;}


    }


    tip_coordinate = tip_from_FK;

    std::cout << "tip_coordinate: \n" << tip_coordinate << std::endl;

    std::cout << "solving frozen IK for this tip coordinate.. \n";

    dvrk_inverse.ik_solve_frozen_refined(tip_coordinate);

    q_vec_frozen_ik_refined = dvrk_inverse.get_soln_frozon_ik_refined();

    std::cout << "q_vec_frozen_ik_refined: \n" << q_vec_frozen_ik_refined << std::endl;

    affine_gripper_wrt_base_frozen = dvrk_forward.fwd_kin_solve(q_vec_frozen_ik_refined);


    std::cout << "TARGET FROZEN tip_coordinate: \n" << tip_coordinate << std::endl << std::endl;
    std::cout << "affine_gripper_wrt_base_frozen.translation: \n" << affine_gripper_wrt_base_frozen.translation() << std::endl;
    std::cout << "affine_gripper_wrt_base_frozen.linear: \n" << affine_gripper_wrt_base_frozen.linear() << std::endl << std::endl;
    std::cout << "affine_gripper_wrt_base.translation: \n" << affine_gripper_wrt_base.translation() << std::endl;
    std::cout << "affine_gripper_wrt_base.linear: \n" << affine_gripper_wrt_base.linear() << std::endl;
  }



  double rate_0 = (entry_count - err_cnt)/double(entry_count);
  double rate_1 = (entry_count - err_cnt_refined)/double(entry_count);

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::cout << std::endl << "\e[1m--- --- --- --- --- --- IK REFINED TEST REPORT --- --- --- --- --- ---\e[0m" << std::endl ;
  std::cout << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << std::endl << std::endl;
  std::cout << "Total trial number: " << entry_count << std::endl;
  std::cout << "Without additional Jacobian correction, \e[31m\e[1m" << err_cnt << " \e[0mof them failed the error assessment*." << std::endl;
  std::cout << "After adding the Jacobian addon, \e[31m\e[1m" << err_cnt_refined << " \e[0mof them did NOT pass." << std::endl;
  std::cout.precision(6);
  std::cout << "Success rate has been raised from \e[4m" << rate_0 << "\e[0m to \e[4m" << rate_1 << "\e[0m"<< std::endl;

  std::cout << std::endl << " * error assessment: " << std::endl
    << "   Only the translational error (the norm of the tip coordinates of the desired pose and the one obtained from the IK.):"
    << std::endl
    << "   tip_err.norm() > 0.001"
    << std::endl;


  // printf("err_cnt = %d", err_cnt);
  std::cout << std::endl;
  return 0;
  }
