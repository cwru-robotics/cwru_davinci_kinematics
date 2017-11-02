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
 * @brief The Vectorq6x1 and Vectorq7x1 are used to save space and act as the SE(3) transform space and jointspace  states respectively.
 * The 
 */
typedef Eigen::Matrix<double, 6, 1> Vectorq6x1;
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
   * @brief Randomly generate a set of legal joint angles for the dvrk.
   *
   * The limits of the joints are defined in the davinci_kinamatic_definiitons.h header file.
   *
   * @param qvec is the generated joint list
   */
  static void gen_rand_legal_jnt_vals(Vectorq7x1 &qvec);

  static bool get_jnt_val_by_name(std::string jnt_name, sensor_msgs::JointState jointState, double &qval);

  // TODO(rcj): replace the dependence on these functions with the tf2_eigen library.
  // http://docs.ros.org/jade/api/tf_conversions/html/c++/tf__eigen_8h.html
  static Eigen::Affine3f transformTFToEigen(const tf::Transform &t);
  static Eigen::Affine3f stampedTFToEigen(const tf::StampedTransform &t);
  static Eigen::Affine3d transformTFToAffine3d(const tf::Transform &t);
  static Eigen::Affine3d stampedTFToAffine3d(const tf::StampedTransform &t);
  static geometry_msgs::Pose transformEigenAffine3fToPose(Eigen::Affine3f e);
  static geometry_msgs::Pose transformEigenAffine3dToPose(Eigen::Affine3d e);

  /**
   * @brief The default constructor of the forward kinematics library.
   */
  Forward();

  /**
   * @brief Given a vector of joint states in DaVinci coords, convert these into
   * equivalent DH parameters, theta and d 
   *
   * @param q_vec the vector of joint states.
   */
  void convert_qvec_to_DH_vecs(const Vectorq7x1& q_vec);

  /**
   * @brief Given two vectors of DH parameters theta and d, compute the joint states in DaVinci coords.
   *
   * @param thetas_DH_vec the vector of DH theta values.
   * @param dvals_DH_vec the vector of DH d values.
   *
   * @return The list of joint states in DaVinci coords.
   *
   * @TODO Identify if this function should be relocated to the inverse kinematics library.
   */
  Vectorq7x1 convert_DH_vecs_to_qvec(const Eigen::VectorXd &thetas_DH_vec, const Eigen::VectorXd &dvals_DH_vec);

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
   * @brief Given the full set of joint positions, compute the SE(3) transform of the  Provide joint angles and prismatic displacement (q_vec[2]) w/rt DaVinci coords
   * 
   * will get translated to DH coords to solve fwd kin
   * return affine describing gripper pose w/rt base frame
   *
   * @param q_vec The joint states of the robot
   *
   * @return The full DaVinci SE(3) transform.
   */
  Eigen::Affine3d fwd_kin_solve(const Vectorq7x1& q_vec);

  /**
   * @brief The following function takes args of DH thetas and d's and returns an affine transformation
   *
   * @param theta_vec The vector of DH theta values.
   * @param d_vec The vector of DH q values.
   *
   * @return The full DaVinci SE(3) transform.
   */
  Eigen::Affine3d fwd_kin_solve_DH(const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& d_vec);

  /**
   * @brief Compute a 6x6 Jacobian derivative of the DaVinci's position transform.
   *
   * It is assumed that the first 5 joints plus angle of pt between fingertips are used as the input joints.
   *
   * @param q_vec The robot joint positions.
   *
   * @return a 6x6 Eigen Matrix. 
   */
  Eigen::MatrixXd compute_jacobian(const Vectorq7x1& q_vec);

  /**
   * @brief Gets the frame from the first i joints (inclusive of joint i).
   *
   * @return the transform through joint i with respect to the base.
   */
  Eigen::Affine3d get_affine_frame(int i)
  {
    return affine_products_[i];
  };

  /**
   * @brief gets the transform from the world frame to the joint frame 0.
   *
   * @return The transform between the world frame and frame0.
   */
  Eigen::Affine3d get_frame0_wrt_base() const;

  /**
   * @brief gets the transform from the joint frame 6 frame to the gripper frame.
   *
   * @return The transform between the joint frame 6 and the gripper frame.
   */
  Eigen::Affine3d get_gripper_wrt_frame6() const;


  double dh_var_to_qvec(double dh_val, int index);

private:
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
   * @brief The list of inter joint transforms (there are 7 total).
   */
  std::vector <Eigen::Affine3d> affines_i_wrt_iminus1_;

  /**
   * @brief This is the list of transforms from the robot base to each joint.
   */
  std::vector <Eigen::Affine3d> affine_products_;

  /**
   * @brief This is the list of theta values of the DaVinci DH joint parameters.
   */
  Eigen::VectorXd thetas_DH_vec_;

  /**
   * @brief This is the list of d values of the DaVinci DH joint parameters.
   */
  Eigen::VectorXd dvals_DH_vec_;

  /**
   * @brief This is the list of theta offset values for the DaVinci DH joint parameters.
   */
  Eigen::VectorXd theta_DH_offsets_;

  /**
   * @brief This is the list of d offset values for the DaVinci DH joint parameters.
   */
  Eigen::VectorXd dval_DH_offsets_;

  /**
   * @brief The 6x6 Jacobian of the current joint positions, only computed when the compute_jacobian method is called.
   */
  Eigen::MatrixXd Jacobian_;
};

}  // namespace davinci_kinematics

#endif  // CWRU_DAVINCI_KINEMATICS_DAVINCI_FWD_KINEMATICS_H
