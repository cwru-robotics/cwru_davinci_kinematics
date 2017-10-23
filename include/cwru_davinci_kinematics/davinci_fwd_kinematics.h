/*
 *  davinci_fwd_kinematics.h
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

// NOTE:  FK and IK assume that the gripper-tip frame is expressed with respect
// to the respective PMS base frame (not camera frame).  For motions w/rt camera
//  first transform the desired camera-frame pose into base-frame pose.

#ifndef CWRU_DAVINCI_KINEMATICS_DAVINCI_FWD_KINEMATICS_H
#define CWRU_DAVINCI_KINEMATICS_DAVINCI_FWD_KINEMATICS_H

#include <vector>
#include <Eigen/Eigen>
#include <string>

#include <tf/transform_listener.h>
#include <sensor_msgs/JointState.h>
#include <cwru_davinci_kinematics/davinci_kinematic_definitions.h>

/**
 * @brief The davinci_kinematics namespace is where both the forward and inverse dvrk kinematic libraries are defined
 *  
 * This is to prevent namespace collisions.
 */
namespace davinci_kinematics
{

/**
 * @brief The Vectorq7x1 is used to save space and act as the SE(3) transform space and jointspace states respectively.
 */
typedef Eigen::Matrix<double, 7, 1> Vectorq7x1;


/**
 * @brief The Forward kinematics library definition. 
 *
 * In addition to computing the SE(3) transform based on a set of joint angles,
 * This object also generates a Jacobian matrix.
 */
class Forward
{
public:
   /**
    * @brief check joint limits against the kinematics to see if they are valid.
    *
    * @param q_vec The vector of proposed joint angles
    *
    * @return zero if all the joints are legal. Otherwise returns a code indicating how the constraints are violated.
    *
    * Code Outline:
    * < 0: A binary indication of which joints are in violation: [-127, -1].
    *   1: While technically reachable, this joint vector places the wrist inside the cannula.
    *   2: The violation is due to the gripper jaw being open past its limits. The limits are dependant on joint index 5. 
    */
  static int check_jnts(const Vectorq7x1& q_vec);

  /**
   * @brief Randomly generate a set of legal joint angles for the dvrk.
   *
   * The limits of the joints are defined in the davinci_kinamatic_definiitons.h header file.
   *
   * @param qvec is the generated joint list
   */
  static void gen_rand_legal_jnt_vals(Vectorq7x1 &qvec);

  /**
   * @brief Get the value of a joint angle by name
   *
   * 
   */
  static bool get_jnt_val_by_name(std::string jnt_name, sensor_msgs::JointState jointState, double &qval);

  /**
   * @brief The default constructor of the forward kinematics library.
   */
  Forward();

  /**
   * @brief Given the full set of joint positions, compute the SE(3) transform of the  Provide joint angles and prismatic displacement (q_vec[2]) w/rt DaVinci coords
   * 
   * will get translated to DH coords to solve fwd kin
   * return affine describing gripper pose w/rt base frame
   *
   * @param q_vec (optional) The joint states of the robot.
   *
   * @return The full DaVinci SE(3) transform.
   */
  Eigen::Affine3d fwd_kin_solve(const Vectorq7x1& q_vec);
  Eigen::Affine3d fwd_kin_solve();

  /**
   * @brief Return the current state of the forward kinematic model
   * @return the SE(3) transform of the forward kinematics.
   */
  Eigen::Affine3d get_fwd_kin();

  /**
   * @brief Return the current state of the forward kineamtic joints
   */
  Vectorq7x1 get_fwd_joints();

  /**
   * @brief Compute a 6x6 Jacobian derivative of the DaVinci's position transform.
   *
   * It is assumed that the first 6 joints plus angle of pt between fingertips are used as the input joints.
   *
   * @param q_vec (optional) The robot joint positions. If empty, the current joints are used for computation.
   *
   * @return a 6x6 Eigen Matrix. 
   */
  Eigen::MatrixXd compute_jacobian(const Vectorq7x1& q_vec);
  Eigen::MatrixXd compute_jacobian();

  /**
   * @brief gets the transform from the world frame to the joint frame 0.
   *
   * @return The transform between the world frame and frame0.
   */
  Eigen::Affine3d get_frame0_wrt_base() const;

  /**
   * @brief sets the transform from the base frame to the joint frame 0.
   *
   * @param affine_frame0_wrt_base The new transform between the world frame and frame0.
   */
  void set_frame0_wrt_base(const Eigen::Affine3d &affine_frame0_wrt_base);

  /**
   * @brief gets the transform from the joint frame 6 frame to the gripper frame.
   *
   * @return The transform between the joint frame 6 and the gripper frame.
   */
  Eigen::Affine3d get_gripper_wrt_frame6() const;

  /**
   * @brief sets the transform from the joint frame 6 frame to the gripper frame.
   *
   * @param jaw_length the length of the gripper jaw.
   * 
   * @return The transform between the joint frame 6 and the gripper frame.
   */
  void set_gripper_jaw_length(double jaw_length);

  /**
   * @brief returns the length of the gripper jaw.
   *
   * @return the set jaw length.
   */
  double get_gripper_jaw_length() const
  {
    return gripper_jaw_length_;
  }

protected:
  /**
   * @brief The following function takes args of DH thetas and d's and returns an affine transformation
   *
   * @param theta_vec The vector of DH theta values.
   * @param d_vec The vector of DH q values.
   *
   * @return The full DaVinci SE(3) transform.
   */
  void fwd_kin_solve_DH(const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& d_vec);

  /**
   * @brief Given two vectors of DH parameters theta and d, compute the joint states in DaVinci coords.
   *
   * @param thetas_DH_vec the vector of DH theta values.
   * @param dvals_DH_vec the vector of DH d values.
   *
   * @return The list of joint states in DaVinci coords.
   */
  Vectorq7x1 convert_DH_vecs_to_qvec(const Eigen::VectorXd &thetas_DH_vec, const Eigen::VectorXd &dvals_DH_vec);

  /**
   * @brief Gets the frame from the first i joints (inclusive of joint i).
   *
   * @param i is the index of the frame wrt frame 0.
   *
   * @return the transform through joint i with respect to frame0 of the robot (may be different than base).
   */
  Eigen::Affine3d get_affine_frame(int i)
  {
    return affine_frame0_wrt_base_.inverse() * affine_products_[i];
  };
private:
  /**
   * @brief Given a vector of joint states in DaVinci coords, convert these into
   * equivalent DH parameters, theta and d 
   *
   * @param q_vec the vector of joint states.
   *
   * @param thetas_DH_vec the resulting DH theta parameters.
   * @param dvals_DH_vec the resulting d values of the theta parameters.
   */
  void convert_qvec_to_DH_vecs(const Vectorq7x1& q_vec, Eigen::VectorXd &thetas_DH_vec, Eigen::VectorXd &dvals_DH_vec);

  /**
   * @brief Given a full set of DH parameters, compute the Eigen Affine transform.
   *
   * This Affine transform is also a member of the SE(3) subspace of affine transforms.
   *
   * @param a The a value of the DH parameter.
   * @param d The d value of the DH parameter.
   * @param alpha The alpha value of the DH parameter.
   * @param theta The theta value of the DH parameter.
   *
   * @return The SE(3) transformation computed from the input DH parameters.
   */
  Eigen::Affine3d computeAffineOfDH(double a, double d, double alpha, double theta);

  /**
   * @brief convert the dh  joint parameter to the joint value for a specific joint index.
   *
   * @param dh_val the dh value
   * @param index The index of the joint (0-6)
   *
   * @return the joint value
   */
  double dh_var_to_qvec(double dh_val, int index);

  /**
   * @brief This is the internal joint state of the forward kinematics.
   */
  Vectorq7x1 current_joint_state_;

  /**
   * @brief This is the tranform aligning the robot with a world frame.
   */ 
  Eigen::Affine3d affine_frame0_wrt_base_;

  /**
   * @brief This is the tranform between the last joint and the robot gripper tip.
   */
  Eigen::Affine3d affine_gripper_wrt_frame6_;

  /**
   * @brief This is the tranform between the gripper and the robot base.
   *
   * recomputed when a new set of joints are passed in.
   */
  Eigen::Affine3d affine_gripper_wrt_base_;

  /**
   * @brief This is the list of transforms from the robot base to each joint.
   *
   * recomputed when a new set of joints are passed in.
   */
  std::vector <Eigen::Affine3d> affine_products_;

  /**
   * @brief This is the list of theta offset values for the DaVinci DH joint parameters.
   * 
   * Stored intrinsicaly as part of the dvrk kinematics
   */
  Eigen::VectorXd theta_DH_offsets_;

  /**
   * @brief This is the list of d offset values for the DaVinci DH joint parameters.
   * 
   * Stored intrinsicaly as part of the dvrk kinematics
   */
  Eigen::VectorXd dval_DH_offsets_;

  /**
   * @brief The 6x6 Jacobian of the current joint positions, only computed when the compute_jacobian method is called.
   *
   * Retained after computation
   */
  Eigen::MatrixXd Jacobian_;

  /**
   * @brief the length of the gripper jaw.
   */
  double gripper_jaw_length_;
};

}  // namespace davinci_kinematics

#endif  // CWRU_DAVINCI_KINEMATICS_DAVINCI_FWD_KINEMATICS_H
