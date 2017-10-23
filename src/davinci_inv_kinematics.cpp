/*
 *  davinci_inv_kinematics.cpp
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

#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include <string>
#include <vector>

/**
 * TODO(wsn) There are alot of questions and queries in the the comments.
 * TODO(rcj) Validate that the questions have been answered.
 */
namespace davinci_kinematics
{

Inverse::Inverse() : Forward()
{
  // Blank constructor.
}

Eigen::Vector3d Inverse::q123_from_wrist(Eigen::Vector3d wrist_pt)
{
  // TODO(wsn) There is ambiguitity in that d3 might be negative.
  // TODO(wsn) Implement a method for detecting if the wrist is inside the cannula.
  double d3 = wrist_pt.norm();
  // now, w = R_1/0*R_2/1*[0;0;d3]
  // or, [wx;wy;wz] = [c1*s2;s1*s2; -c2]*d3

  // transform w to w_wrt_frame0, then scale it w/ w/d3;
  // note: in frame0, wrist-point z-value is measured along yaw (jnt1) z-axis;
  // displacement along z0 axis depends on tool-insertion length, d3, and on rotation of pitch mechanism, theta2
  // note that theta2 is pi/2 + q_davinci(1);
  // if range of q_davinci is +/- pi/2, then range of theta2 is 0 to +pi

  Eigen::Vector3d w_prime = wrist_pt / d3;

  // arc cosine of x, in the interval [0,pi] radians...which is interval of interest for theta2, so keep this soln
  double theta2;
  theta2 = acos(-w_prime(2));
  // s2 will always be >0 for 0<theta2<pi
  // so atan2 should yield a good answer

  double theta1;
  theta1 = atan2(w_prime(1), w_prime(0));

  Eigen::Vector3d q123;
  q123(0) = theta1;
  q123(1) = theta2;
  q123(2) = d3;

  return q123;
}

// defined tool-tip frame such that x-axis is anti-parallel to the gripper-jaw rotation axis
// "5" frame is frame w/ z-axis through the last rotation joint--rotation of gripper jaws
// return the wrist point...but also calculate zvec_4
//  zvec_4 has a +/- ambiguity
void Inverse::compute_w_from_tip(Eigen::Affine3d affine_gripper_tip,
  Eigen::Vector3d &zvec_4a, Eigen::Vector3d &zvec_4b,
  Eigen::Vector3d &sol_O4a, Eigen::Vector3d &sol_O4b)
{
  // the following are all expressed w/rt the 0 frame
  Eigen::Vector3d zvec_tip_frame, xvec_tip_frame, origin_5, zvec_5, xvec_5, origin_4;
  Eigen::Matrix3d R_tip;
  R_tip = affine_gripper_tip.linear();
  zvec_tip_frame = R_tip.col(2);
  xvec_tip_frame = R_tip.col(0);
  // by definition of tip frame
  // equation (1) from [1]
  zvec_5 = -xvec_tip_frame;

  origin_5 = affine_gripper_tip.translation() - this->get_gripper_jaw_length()*zvec_tip_frame;

  // consider these two planes:
  // define vector z_perp, which is the same a z5
  // P_perp contains O5 and is perpendicular to z5; claim: P_perp contains 04
  // P_parallel is defined by: contains O_0, contains O_5 and contains z5; claim: P_parallel contains O4
  // given P_parallel, can compute the normal vector to this plane--call it z_parallel
  //
  // note that plane P_parallel is perpendicular to P_perp; (z_perp is perpendicular to z_parallel)
  // if both planes contain O4, then O4 lies along the line of intersection of P_perp with P_parallel
  // this line must be perpendicular to z_perp and to z_parallel, and thus it is +/- z_parallel cross z_perp
  // call this intersect_vec;
  // intersect_vec is the same as +/- x5: the vector from z4 to z5 (in DH notation); sign is ambiguous at this point
  // O4 can be found by starting from O5, moving distance "dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis" along x5
  // to resolve the sign ambiguity, consider two O4 candidates: O4a = O5-dist*intersect_vec,
  // and O4b = O5+dist*intersect_vec
  // the correct solution is the point that is CLOSEST to the origin O_0
  // TODO(rcj, wsn) All of these ambiguitities cause problems. Can we decisively remove them?
  // plane P_perp is perpendicular to z_perp and contains O5
  // plane P_parallel is perpendicular to z_parallel and contains O5, base origin, and z_perp
  // used to define a plane perpendicular to jaw-rotation axis

  // z_parallel is a problem if zvec_5 points at the origin (portal).
  // worst case of feasible pose is when q5 is +/- 90 deg.
  // O5 - O_0 is same as O5

  // could be + or -  ?
  xvec_5 = zvec_5.cross(zvec_5.cross(origin_5));

  if (xvec_5.norm() < 0.001)
  {
    printf("There is an issue in computing the x_5 direction vector");
  }
  // should not be necessary--already unit length
  xvec_5 = xvec_5/(xvec_5.norm());

  // should get gripper-jaw angle from gripper z_des and xvec_5
  // q6 is rotation from xvec_5 to z_gripper_des about zvec_5


  sol_O4a = origin_5 - dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis*xvec_5;
  sol_O4b = origin_5 + dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis*xvec_5;

  // possible error here: need to get sign of xvec_5 correct.
  // given O_4 and O_5, should have xvec_5 point from O_4 towards O_5
  // if using CORRECT direction of x5 axis and z5 axis, does CORRECT direction of zvec_4 follow?
  // ambiguity here: zvec_4 could be +/- along this direction
  // zvec_4 = yvec_4
  zvec_4b = (zvec_5.cross(xvec_5));
  zvec_4a = -(zvec_5.cross(xvec_5));
}

bool Inverse::fit_q_to_range(double q_min, double q_max, double &q)
{
  q = fmod(q, 2.0 * M_PI);

  if (q > M_PI)
  {
    q -= 2.0 * M_PI;
  }
  if (q < -M_PI)
  {
    q += 2.0 * M_PI;
  }
  if (q <= q_min || q >= q_max)
  {
    // printf("Range is <%f, %f>, joint is: %f\n", q_min, q_max, q);
    return false;
  }
  else
    return true;
}

bool Inverse::fit_joints_to_range(Vectorq7x1 &qvec)
{
  bool fits = true;
  bool does_fit;
  double q;
  for (int i = 0; i < 7; i++)
  {
    q = qvec[i];
    // treat d3 differently since it is a translational joint.
    // special case for d3...although generic formula also works in this case
    if (i != 2)
    {
      does_fit = fit_q_to_range(q_lower_limits[i], q_upper_limits[i], q);
      if (does_fit == false)
      {
       // printf("Joint %d failed\n", i);
      }
    }
    else
    {
      does_fit = true;
      if (q < q_lower_limits[i])
      {
        // printf("Linear joint %d value: %f, lower limit: %f\n", i, q, q_lower_limits[i]);
        does_fit = false;
      }
      if (q > q_upper_limits[i])
      {
        // printf("Linear joint %d value: %f, upper limit: %f\n", i, q, q_upper_limits[i]);
        does_fit = false;
      }
    }
    qvec[i] = q;
    fits = fits && does_fit;
  }
  if (fits)
    return true;
  else
    return false;
}

int Inverse::ik_solve(Eigen::Affine3d const& desired_hand_pose)
{
  // before doing anything else, premultiply to get everything in terms of the base.
  Eigen::Affine3d affine_frame0_wrt_base = this->get_frame0_wrt_base();
  desired_hand_pose_ = affine_frame0_wrt_base.inverse() * desired_hand_pose;
  // desired_hand_pose_ = desired_hand_pose;

  Eigen::Vector3d z4_wrt_3, O_6_wrt_4, xvec6_wrt_5, O_5_wrt_base, zvec5_wrt_base;
  Eigen::Vector3d des_tip_origin, zvec_tip_wrt_base;
  Eigen::VectorXd theta_vec, d_vec;
  Eigen::Matrix3d R_tip_wrt_base;

  q_vec_soln_(0) = -10.0;
  q_vec_soln_(1) = -10.0;
  q_vec_soln_(2) = -10.0;
  q_vec_soln_(3) = -10.0;
  q_vec_soln_(4) = -10.0;
  q_vec_soln_(5) = -10.0;
  q_vec_soln_(6) = -10.0;

  // TODO(rcj, wsn) Look through these error codes as some legal joint definitions may result in constraint violations.
  // TODO(wsn, rcj) add the error definitions to the readme.
  // des_tip_origin = desired_hand_pose_.translation();
  // double tool_tip_z_des = des_tip_origin(2);
  // TODO(rcj) I propose removing this condition from inv kinematics OR adding it to fwd kinematics.
  /*if (tool_tip_z_des > 0.0)
  {
    // Disallow a positive tool-tip z-height since that would be above the portal
    // in fact, must insert at least past the wrist joint, z4, so
    // tip_z must be at least...?
    // return 0;
    return -1;
  }*/

  R_tip_wrt_base = desired_hand_pose_.linear();
  zvec_tip_wrt_base = R_tip_wrt_base.col(2);
  O_5_wrt_base = des_tip_origin - zvec_tip_wrt_base * gripper_jaw_length;
  // This should be 0.00
  // TODO(rcj) I propose the same here as in the tool_tip_z_des (can these be combined).
  /*if (O_5_wrt_base(2) > 0.00)
  {
    // If O5 is above the portal, there are no solutions:
    printf("The offset value is: %f\n", O_5_wrt_base(2));
    std::cout << des_tip_origin << std::endl << std::endl;
    std::cout << zvec_tip_wrt_base << std::endl << std::endl;
    std::cout << gripper_jaw_length << std::endl << std::endl;
    std::cout << desired_hand_pose.linear() << std::endl << std::endl;
    return -2;
  }

  double projection_gripper_zvec_onto_O5_vec = zvec_tip_wrt_base.dot(O_5_wrt_base);
  if (projection_gripper_zvec_onto_O5_vec <= 0.0)
  {
    // test if gripper z-axis implies excessive wrist bend:
    // consider vector from portal (origin) to O5 (jaws-axis), and project
    // the desired gripper z-axis onto this vector. Result must be > 0 for
    // wrist bend to be |q5|< pi/2
    // this is a necessary but not sufficient test;
    // can still violate wrist-bend>pi/2 and pass this test
    return -3;
  }
  // by definition of tip frame
  // better: look at cross product of O_5_wrt_base and zvec5_wrt_base
  // gripper x-axis is same as z5
  zvec5_wrt_base = -R_tip_wrt_base.col(0);
  double mag_z5xO5 = (zvec5_wrt_base.cross(O_5_wrt_base)).norm();
  // had to soften this clause as well.
  if ((mag_z5xO5 + 0.0001) < (dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis))
  {
    std::cout << mag_z5xO5 << std::endl << std::endl;
    std::cout << dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis << std::endl << std::endl;
    return -4;
  } */

  // first step: get the wrist-bend origin on tool shaft from desired gripper pose:
  // desired_hand_pose input, z_vec4, alt_w_wrt_base output
  // TODO(rcj, wsn) If possible resolve the ambiguity in compute_w_from_tip.
  Eigen::Vector3d w_wrt_base[2];
  Eigen::Vector3d z_vec4[2];
  compute_w_from_tip(desired_hand_pose_, z_vec4[0], z_vec4[1], w_wrt_base[0], w_wrt_base[1]);

  // next step: get theta1, theta2, d3 soln from wrist position:

  std::vector<Vectorq7x1> q_sol;
  std::vector<double> err_l;
  std::vector<double> err_r;
  std::vector<Vectorq7x1> q_fail;

  for (int index(0); index < 4; index++)
  {
    int index_2(index >> 1);
    int index_1(index % 2);

    Eigen::Vector3d q123(q123_from_wrist(w_wrt_base[index_1]));
    Vectorq7x1 q_sol_p = compute_q456(q123, z_vec4[index_2]);

    if (fit_joints_to_range(q_sol_p))
    {
      q_sol.push_back(q_sol_p);
      // compute the numerical errors.
      Eigen::Affine3d affine_test_fk = fwd_kin_solve(q_sol_p);
      Eigen::Matrix3d fwd_inv = affine_test_fk.rotation() * desired_hand_pose_.rotation().inverse();
      Eigen::Quaterniond fwd_inv_q(fwd_inv);
      err_l.push_back((affine_test_fk.translation() - desired_hand_pose_.translation()).norm());
      err_r.push_back(acos(fwd_inv_q.w()) * 2);
    }
    else
    {
      q_fail.push_back(q_sol_p);
    }
  }

  switch (q_sol.size())
  {
    case 0:
    {
    }
    return -6;

    case 1:
    {
      q_vec_soln_ = q_sol[0];
      err_l_ = err_l[0];
      err_r_ = err_r[0];
    }
    return 1;

    default:
    {
      std::cout << "There are multiple solutions\n";

      for (unsigned int index(0); index < q_sol.size(); index++)
      {
        printf("Solution %d has errors of <%f, %f> and is: \n", index, err_l[index], err_r[index]);
        std::cout << q_sol[index] << "\n\n";
      }
    }
    return q_sol.size();
  }
  // This is logically unreachable.
  return -7;
}

Eigen::Vector3d Inverse::compute_fk_wrist(Eigen::Vector3d q123)
{
  Eigen::Affine3d affine_frame_wrt_base;
  Eigen::Vector3d wrist_pt;
  Eigen::VectorXd theta_vec, d_vec;

  // compute FK of this soln:
  theta_vec.resize(7);
  theta_vec << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  theta_vec(0) = q123(0);
  theta_vec(1) = q123(1);

  d_vec.resize(7);
  d_vec << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  d_vec(2) = q123(2);

  // use partial IK soln to compute FK of first three frames:
  fwd_kin_solve_DH(theta_vec, d_vec);
  affine_frame_wrt_base = get_affine_frame(2);
  wrist_pt = affine_frame_wrt_base.translation();
  return wrist_pt;
}

Vectorq7x1 Inverse::compute_q456(Eigen::Vector3d q123, Eigen::Vector3d z_vec4)
{
  Eigen::Affine3d affine_frame_wrt_base, affine_frame6_wrt_4, affine_frame6_wrt_5, fk_gripper_frame;
  Eigen::Vector3d z4_wrt_3, O_6_wrt_4, xvec6_wrt_5;
  Eigen::VectorXd theta_vec, d_vec;

  // Eigen::Affine3d affine_frame0_wrt_base = this->get_frame0_wrt_base();
  // Eigen::Affine3d affine_base_wrt_frame0 = affine_frame0_wrt_base.inverse();

  theta_vec.resize(7);
  theta_vec << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  theta_vec(0) = q123(0);
  theta_vec(1) = q123(1);

  d_vec.resize(7);
  d_vec << 0, 0, 0, 0, 0, 0, 0;
  d_vec(2) = q123(2);

  // use partial IK soln to compute FK of first three frames:
  fwd_kin_solve_DH(theta_vec, d_vec);

  // this frame depends only on 1st 3 var's
  affine_frame_wrt_base = get_affine_frame(2);
  Eigen::Matrix3d R_3_wrt_base;
  R_3_wrt_base = affine_frame_wrt_base.linear();

  // Express z_vec4 in frame-3 coords. Expect z-component to be zero.
  z4_wrt_3 = R_3_wrt_base.transpose() * z_vec4;

  // TODO(wsn) since there is ambiguity of +/- z_vec4, there are 2 solns which are PI apart
  // TODO(wsn) Implement a method for resolving this.
  double theta4 = atan2(z4_wrt_3(1), z4_wrt_3(0)) + M_PI/2.0;
  // for the following, it might be easier to use knowledge of O4 and O5 to compute theta5
  // also, given x5_vec, and z6_vec_desired, should be able to get theta6

  // recompute FK for 1st 4 variables:
  theta_vec(3) = theta4;
  fwd_kin_solve_DH(theta_vec, d_vec);
  // get frame 4, which depends on 1st 4 vars:
  affine_frame_wrt_base = get_affine_frame(3);

  // compute transform frame 6 wrt frame 4:
  // A_{g/base} = A_{4/base}*A_{6/4}*A_{g/6}
  // so, A_{4/base}_inv * A_{g/base} * A_{g/6}_inv = A_{4/base}
  Eigen::Affine3d affine_gripper_wrt_frame6 = this->get_gripper_wrt_frame6();
  affine_frame6_wrt_4 = affine_frame_wrt_base.inverse() * desired_hand_pose_*affine_gripper_wrt_frame6.inverse();
  O_6_wrt_4 = affine_frame6_wrt_4.translation();
  double theta5 =  atan2(O_6_wrt_4(1), O_6_wrt_4(0));


  theta_vec(4) = theta5;
  fwd_kin_solve_DH(theta_vec, d_vec);
  // get frame 5, which depends on 1st 5 vars:
  affine_frame_wrt_base = get_affine_frame(4);
  affine_frame6_wrt_5 = affine_frame_wrt_base.inverse() * desired_hand_pose_ * affine_gripper_wrt_frame6.inverse();

  xvec6_wrt_5 = affine_frame6_wrt_5.linear().col(0);
  double theta6 = atan2(xvec6_wrt_5(1), xvec6_wrt_5(0));
  theta_vec(5) = theta6;

  // pack the solution into a single vector
  return convert_DH_vecs_to_qvec(theta_vec, d_vec);
}

}  // namespace davinci_kinematics
