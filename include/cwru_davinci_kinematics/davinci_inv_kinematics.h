/*
 *  davinci_inv_kinematics.h
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


/**
 * @brief The inverse kinematics class (derived from the forward kinematics class) is for
 * Computing an analytical inverse kinematics of the DaVinci robot.
 */

#ifndef CWRU_DAVINCI_KINEMATICS_DAVINCI_INV_KINEMATICS_H
#define CWRU_DAVINCI_KINEMATICS_DAVINCI_INV_KINEMATICS_H


#include <vector>
#include <Eigen/Eigen>
#include <string>
#include <math.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>

#include <cwru_davinci_kinematics/davinci_fwd_kinematics.h>

namespace davinci_kinematics
{
// RN 20/07/18 It used to be private inheritance.
class Inverse:public Forward
{
public:
  /**
   * @brief The default constructor.
   */
  Inverse();

  /**
   * @brief Get the inverse kinematics of the DaVinci.
   *
   * TODO(rcj,wsn) redefine the error codes and discuss jaw opening.
   * Multiple solutions may be found. If no solutions are found, then the return code indicates the reason for no solutions.
   * -1: tool_tip_z_des <= 0.0
   * -2: O_5_wrt_base(2) <= 0.0
   * -3: projection_gripper_zvec_onto_O5_vec > 0.0
   * -4: mag_z5xO5 >= dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis
   * -5: A joint is out of range.
   *
   *
   * @param desired_hand_pose The desired hand base transform of the  DaVinci robot.
   *
   * @return the number of solutions
   *
   */
  
  [[deprecated]]
  int  ik_solve(Eigen::Affine3d const& desired_hand_pose);

  // RN
  int ik_solve_refined(Eigen::Affine3d const& desired_hand_pose);
  bool solve_jacobian_ik(Eigen::Affine3d const& desired_hand_pose, Eigen::VectorXd &q_ik);
  int ik_solve_frozen_refined(Eigen::Vector3d const& desired_tip_coordinate);
  int ik_solve_frozen_refined(Eigen::Affine3d const& desired_hand_pose);
  bool solve_jacobian_frozen_ik(Eigen::Vector3d const& desired_tip_coordinate, Eigen::VectorXd &q_ik);


  int ik_solve(Eigen::Affine3d const& desired_hand_pose, std::string kinematic_set_name); // is this necessary?
  int ik_solve_generic(Eigen::Affine3d const& desired_hand_pose);

  int ik_solve_refined(Eigen::Affine3d const& desired_hand_pose,
                       std::string kinematic_set_name);

  bool solve_jacobian_ik(Eigen::Affine3d const& desired_hand_pose,
                         Eigen::VectorXd &q_ik,
                         std::string kinematic_set_name);

  int ik_solve_frozen_refined(Eigen::Vector3d const& desired_tip_coordinate,
                              std::string kinematic_set_name);

  int ik_solve_frozen_refined(Eigen::Affine3d const& desired_hand_pose,
                              std::string kinematic_set_name);

  bool solve_jacobian_frozen_ik(Eigen::Vector3d const& desired_tip_coordinate,
                                Eigen::VectorXd &q_ik,
                                std::string kinematic_set_name);


  void resetAllIkMaps();


  /**
   * @brief get the properly computed (and validated) solution.
   *
   * @return the valid solution of the IK.
   */
  Vectorq7x1 get_soln() const
  {
    return q_vec_soln_;
  };

  // RN
  Vectorq7x1 get_soln_refined() const
  {
    return q_vec_soln_refined_;
  };

  // TODO why this cannot add const at the end like other functions?
  Vectorq7x1 get_soln_refined(std::string kinematic_set_name)
  {
    return q_vec_soln_refined_map_[kinematic_set_name];
  };

  void get_soln_refined(std::string kinematic_set_name, std::vector<double>& in)
  {
    Vectorq7x1 tmp = get_soln_refined(kinematic_set_name);
    for(std::size_t i = 0; i < in.size(); i++){
      in[i] = tmp[i];
    }
  };

  // RN
  Vectorq7x1 get_soln_frozon_ik_refined() const
  {
    return q_vec_soln_frozon_ik_refined_;
  };

  Vectorq7x1 get_soln_frozon_ik_refined(std::string kinematic_set_name)
  {
    return q_vec_soln_frozon_ik_refined_map_[kinematic_set_name];
  };

  /**
   * @brief gets the positional error of the inverse kinematics computations
   */
  double getError_l()
  {
    return err_l_;
  }

  /**
   * @brief gets the rotational error of the inverse kinematics computations
   */
  double getError_r()
  {
    return err_r_;
  }

  using Forward::get_frame0_wrt_base;
  using Forward::set_frame0_wrt_base;

  using Forward::get_gripper_wrt_frame6;
  using Forward::set_gripper_jaw_length;

private:

    bool debug_print = false;
    bool debug_print_1 = false;



//  Forward davinci_fwd_solver_;

  /**
   * @brief verifies that the proposed list of joint positions fit the hardware joint limits.
   *
   * @param qvec A modifiable vector of joint angles.
   *
   * @return true if the joints are within the hardware limits.
   */
  bool fit_joints_to_range(Vectorq7x1 &qvec);

  /**
   * @brief Computes the solution to the wrist point for joints 1-3.
   *
   * compute_w_from_tip finds the wrist point (at wrist bend), given desired gripper frame;
   * q123_from_wrist solves for first three joint displacements, given wrist point

   * given a 3-D wrist point w/rt base frame (at portal origin), solve for theta1, theta2 and d3;
   * return these in a vector (in that order)
   * "wrist point" is ambiguous.  meaning here is O_3 = O_4 = intersection of tool-shaft rotation and
   * (first) wrist-bend axis
   */
  Eigen::Vector3d q123_from_wrist(Eigen::Vector3d wrist_pt);

  /**
   * @brief Computes the forward kinematics using only the first 3 joints.
   *
   * debug fnc: compute FK of wrist pt from q123
   * could have put this in IK instead...
   *
   * @param q123 a vector of 3 joint angles. (joints 1-3)
   *
   * @return The wrist point.
   */
  Eigen::Vector3d compute_fk_wrist(Eigen::Vector3d q123);

  /**
   * @brief solve the wrist angles using the proposed position of joints 1-3,
   * The z_vector_4m and the desired hand transform.
   *
   * @param q123 The proposed positons of joints 1-3
   * @param z_vec4 The z direction.
   */
  Vectorq7x1 compute_q456(Eigen::Vector3d q123, Eigen::Vector3d z_vec4);


  /**
   * @brief compute the wrist point based on the z direction and the gripper pose.
   *
   * defined tool-tip frame such that x-axis is anti-parallel to the gripper-jaw rotation axis
   * "5" frame is frame w/ z-axis through the last rotation joint--rotation of gripper jaws
   * return the wrist point...but also calculate zvec_4
   *  zvec_4 has a +/- ambiguity
   *
   * @param affine_gripper_tip The gripper tip transform.
   * @param zvec_4a the first z_vector of the wrist. (output)
   * @param zvec_4b the second z_vector of the wrist. (output)
   * @param sol_04a a origin location. (output)
   * @param sol_04b is an another possible origin location. (output)
   */
  void compute_w_from_tip(Eigen::Affine3d affine_gripper_tip,
    Eigen::Vector3d &zvec_4a, Eigen::Vector3d &zvec_4b,
    Eigen::Vector3d &sol_04a, Eigen::Vector3d &sol_04b);

  /**
   * @brief validate that a joint is inside of its joint limits.
   *
   * @param q_min The minimum joint value allowed.
   * @param q_max The maximum joint value allowed.
   * @param q The proposed joint value (modifiable).
   *
   * @return True if the joint is inside the limits (after wrapping into the range 0-2pi).
   */
  bool fit_q_to_range(double q_min, double q_max, double &q);

  /**
   * @brief The resulting solution of the inverse kinematics computation
   */
  Vectorq7x1 q_vec_soln_;

  /**
   * RN
   */
  Vectorq7x1 q_vec_soln_refined_;

  /**
   * RN
   */
  Vectorq7x1 q_vec_soln_frozon_ik_refined_;

  /**
   * @brief The stored desired pose for which the inverse kinematics has been most recently completed
   */
  Eigen::Affine3d desired_hand_pose_;

  /**
   * @brief The minimum distance from joint 4 to the gripper tip.
   */
  // double min_dist_O4_to_gripper_tip_;


  std::map<std::string, Vectorq7x1> q_vec_soln_refined_map_;
  std::map<std::string, Vectorq7x1> q_vec_soln_map_;
  std::map<std::string, Vectorq7x1> q_vec_soln_frozon_ik_refined_map_;
  std::map<std::string, Eigen::Affine3d> desired_hand_pose_map_; // is thie necessary?

  double err_l_;
  double err_r_;


};

}  // namespace davinci_kinematics

#endif  // CWRU_DAVINCI_KINEMATICS_DAVINCI_INV_KINEMATICS_H
