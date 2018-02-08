// RN

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

  davinci_kinematics::Vectorq7x1 q_vec, err_vec, q_vec_ik, q_vec_ik_refined;
  davinci_kinematics::Vectorq7x1 err_vec_refined_ik;
  q_vec << 0, 0, 0, 0, 0, 0, 0;
  int err_cnt = 0;

  int err_cnt_refined = 0;
  int refined_total_count = 0;
  int entry_count = 0;

  Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base, affine_frame_wrt_base, affine_gripper_wrt_base1;
  Eigen::Affine3d affine_wrist_wrt_base2, affine_gripper_wrt_base2, affine_frame_wrt_base2;
  // wait to start receiving valid tf transforms

  Eigen::Vector3d tip_from_FK, tip_from_FK_of_IK, tip_from_FK_of_refined_IK, tip_err, tip_err_refined;
  // note: with print-outs, takes about 45sec for 10,000 iterations, and got 0 errors
  for (int itries = 0; itries < 200; itries++)
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
      entry_count++;
      q_vec_ik = dvrk_inverse.get_soln();

      printf("FK of IK soln: \n");
      affine_gripper_wrt_base1 = dvrk_forward.fwd_kin_solve(q_vec_ik);
      std::cout << "affine linear (R): " << std::endl;
      std::cout << affine_gripper_wrt_base1.linear() << std::endl;
      tip_from_FK_of_IK = affine_gripper_wrt_base1.translation();
      std::cout << "origin: ";
      std::cout << tip_from_FK_of_IK.transpose() << std::endl;

      std::cout << std::endl;
      // std::cout << "q_vec in: " << q_vec.transpose() << std::endl;
      // std::cout << "q_vec_ik: " << q_vec_ik.transpose() << std::endl;
      err_vec = q_vec - q_vec_ik;

      // std::cout << "err vec: " << err_vec.transpose() << std::endl;
      std::cout << std::endl;
      std::cout << std::endl;
      tip_err = tip_from_FK - tip_from_FK_of_IK;
      // std::cout << "tip err: " << tip_err.transpose() << std::endl;

      // printf("jspace errvec norm: %f\n", err_vec.norm());
      // printf("tip pos err norm: %f:\n", tip_err.norm());
      if (err_vec.norm() + tip_err.norm() > 0.0001)
      {
        printf("excessive error!");
        err_cnt++;
      }
      // printf("itries = %d; err_cnt = %d", itries, err_cnt);

      int refinement_outcome = 9;
      std::cout << std::endl << "refinement_outcome_INIT: " << refinement_outcome << std::endl << std::endl;
      refinement_outcome = dvrk_inverse.ik_solve_refined(affine_gripper_wrt_base);
      // std::cout << std::endl << "dvrk_inverse.ik_solve_refined(affine_gripper_wrt_base);: " << dvrk_inverse.ik_solve_refined(affine_gripper_wrt_base) << std::endl;
      std::cout << std::endl << "refinement_outcome: " << refinement_outcome << std::endl << std::endl;

      if (refinement_outcome > 1)
      {
        refined_total_count ++;

        q_vec_ik_refined = dvrk_inverse.get_soln_refined();
        affine_gripper_wrt_base2 = dvrk_forward.fwd_kin_solve(q_vec_ik_refined);

        tip_from_FK_of_refined_IK = affine_gripper_wrt_base2.translation();

        std::cout << "q_vec in:            " << q_vec.transpose() << std::endl;
        std::cout << "q_vec_ik:            " << q_vec_ik.transpose() << std::endl;
        std::cout << "q_vec_ik_refined:    " << q_vec_ik_refined.transpose() << std::endl;


        err_vec_refined_ik = q_vec - q_vec_ik_refined;
        std::cout << "err vec:             " << err_vec.transpose() << std::endl;
        std::cout << "err_vec_refined_ik:  " << err_vec_refined_ik.transpose() << std::endl;
        std::cout << std::endl;
        tip_err_refined = tip_from_FK - tip_from_FK_of_refined_IK;
        std::cout << "tip err:             " << tip_err.transpose() << std::endl;
        std::cout << "tip_err_refined err: " << tip_err_refined.transpose() << std::endl;
        std::cout << std::endl;

        printf("jspace errvec norm: %f\n", err_vec.norm());
        printf("tip pos err norm: %f:\n", tip_err.norm());
        printf("jspace errvec norm refined: %f\n", err_vec_refined_ik.norm());
        printf("tip pos err norm refined: %f:\n", tip_err_refined.norm());
        std::cout << std::endl;

        if (err_vec_refined_ik.norm() + tip_err_refined.norm() > 0.0001)
        {
          printf("excessive error after refinement!");
          err_cnt_refined++;
        }


        printf("itries = %d; err_cnt = %d", itries, err_cnt);
        std::cout << std::endl << "refined_total_count: " << refined_total_count << std::endl
          << "entry_count: " << entry_count << std::endl;
        std::cout << "err_cnt_refined: " << err_cnt_refined << std::endl;



      } else {std::cout << "NO BETTER SOLUTION!" << std::endl;}



    }

    // dvrk_inverse.ik_solve_refined(affine_gripper_wrt_base);


  }


  std::cout << std::endl << "refined_total_count: " << refined_total_count << std::endl
    << "entry_count: " << entry_count << std::endl;
  std::cout << "err_cnt_refined: " << err_cnt_refined << std::endl;




  printf("err_cnt = %d", err_cnt);
  return 0;
  }
