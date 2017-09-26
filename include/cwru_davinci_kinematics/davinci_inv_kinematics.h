// @TODO Add License Text.
// Copyright Wyatt S. Newman 2015 and Russell Jackson 2017
/* 
 * File:   davinci_kinematics.h
 * Author: wsn
 *
 * Created Sept 2, 2015
 */
// NOTE:  FK and IK assume that the gripper-tip frame is expressed with respect
// to the respective PMS base frame (not camera frame).  For motions w/rt camera
//  first transform the desired camera-frame pose into base-frame pose.

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

/**
 * @brief The inverse kinematics class (derived from the forward kinematics class) is for 
 * computing the inverse kinematics of the DaVinci robot.
 */
class Inverse:private Forward
{
public:
  /**
   * @brief The default constructor.
   */
  Inverse();

  /**
   * @brief Get the inverse kinematics of the DaVinci.
   *
   * Multiple solutions may be found.
   *
   * @param desired_hand_pose The desired hand base transform of the  DaVinci robot.
   *
   * @return the number of solutions
   */
  int  ik_solve(Eigen::Affine3d const& desired_hand_pose);

  /**
   * @brief get the properly computed (and validated) solution.
   *
   * @return the valid solution of the IK.
   */
  Vectorq7x1 get_soln() const
  {
    return q_vec_soln_;
  };

private:
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
   * @param The desired hand_base transform.
   */
  void compute_q456(Eigen::Vector3d q123, Eigen::Vector3d z_vec4, Eigen::Affine3d desired_hand_pose);


  /**
   * @brief compute the wrist point based on the z direction and the gripper pose.
   *
   * defined tool-tip frame such that x-axis is anti-parallel to the gripper-jaw rotation axis
   * "5" frame is frame w/ z-axis through the last rotation joint--rotation of gripper jaws
   * return the wrist point...but also calculate zvec_4
   *  zvec_4 has a +/- ambiguity
   *
   * @param affine_gripper_tip The gripper tip transform.
   * @param zvec_4 the z_vector of the wrist. (output)
   * @param alt_04 an alternative z_vector. (output)
   */
  Eigen::Vector3d compute_w_from_tip(Eigen::Affine3d affine_gripper_tip, Eigen::Vector3d &zvec_4,
    Eigen::Vector3d &alt_O4);

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
   * @brief The minimum distance from joint 4 to the gripper tip.
   */
  double min_dist_O4_to_gripper_tip_;
};

}  // namespace davinci_kinematics

#endif  // CWRU_DAVINCI_KINEMATICS_DAVINCI_INV_KINEMATICS_H
