// @TODO Add License Text.
// Copyright Wyatt S. Newman 2015 and Russell Jackson 2017

#include <math.h>
#include <string>
#include <vector>
#include <cwru_davinci_kinematics/davinci_fwd_kinematics.h>

namespace davinci_kinematics
{

// fnc to extract a joint value from a JointState message;
// provide the name of interest, as a C++ string, and provide the entire
// jointState message;  will set the value of "qval" arg, if possible;
// will return "true" or "false" to indicate if name was found on list
bool Forward::get_jnt_val_by_name(std::string jnt_name, sensor_msgs::JointState jointState, double &qval)
{
  int njnts = jointState.name.size();
  for (int ijnt = 0; ijnt < njnts; ijnt++)
  {
    if (jnt_name.compare(jointState.name[ijnt]) == 0)
    {
      // found a name match!
      qval = jointState.position[ijnt];
      return true;
    }
  }
  // if get to here, did not find a match:
  printf("no match for joint name %s\n", jnt_name.c_str());
  return false;
}

// TODO(rcj, wsn) replace the below utilities with tf2_eigen declared functions

// some utilities to convert data types:
Eigen::Affine3f  Forward::transformTFToEigen(const tf::Transform &t)
{
  Eigen::Affine3f e;
  for (int i = 0; i < 3; i++)
  {
    e.matrix()(i, 3) = t.getOrigin()[i];
    for (int j = 0; j < 3; j++)
    {
      e.matrix()(i, j) = t.getBasis()[i][j];
    }
  }
  // Fill in identity in last row
  for (int col = 0; col < 3; col++)
    e.matrix()(3, col) = 0;

  e.matrix()(3, 3) = 1;
  return e;
}

// as above, but double instead of float
Eigen::Affine3d Forward::transformTFToAffine3d(const tf::Transform &t)
{
  Eigen::Affine3d e;
  for (int i = 0; i < 3; i++)
  {
    e.matrix()(i, 3) = t.getOrigin()[i];
    for (int j = 0; j < 3; j++)
    {
      e.matrix()(i, j) = t.getBasis()[i][j];
    }
  }
  // Fill in identity in last row
  for (int col = 0; col < 3; col++)
    e.matrix()(3, col) = 0;

  e.matrix()(3, 3) = 1;
  return e;
}

// versions for stamped transforms
Eigen::Affine3d Forward::stampedTFToAffine3d(const tf::StampedTransform &t)
{
  tf::Vector3 tf_Origin = t.getOrigin();
  tf::Matrix3x3 tf_R = t.getBasis();

  tf::Transform tf_temp;
  tf_temp.setBasis(tf_R);
  tf_temp.setOrigin(tf_Origin);
  Eigen::Affine3d e;
  e = transformTFToAffine3d(tf_temp);
  return e;
}


Eigen::Affine3f  Forward::stampedTFToEigen(const tf::StampedTransform &t)
{
  tf::Vector3 tf_Origin = t.getOrigin();
  tf::Matrix3x3 tf_R = t.getBasis();

  tf::Transform tf_temp;
  tf_temp.setBasis(tf_R);
  tf_temp.setOrigin(tf_Origin);
  Eigen::Affine3f e;
  e = transformTFToEigen(tf_temp);
  return e;
}

// and go the other direction:
geometry_msgs::Pose Forward::transformEigenAffine3fToPose(Eigen::Affine3f e)
{
  Eigen::Vector3f Oe;
  Eigen::Matrix3f Re;
  geometry_msgs::Pose pose;
  Oe = e.translation();
  Re = e.linear();

  // convert rotation matrix Re to a quaternion, q
  Eigen::Quaternionf q(Re);
  pose.position.x = Oe(0);
  pose.position.y = Oe(1);
  pose.position.z = Oe(2);

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

geometry_msgs::Pose Forward::transformEigenAffine3dToPose(Eigen::Affine3d e)
{
  Eigen::Vector3d Oe;
  Eigen::Matrix3d Re;
  geometry_msgs::Pose pose;
  Oe = e.translation();
  Re = e.linear();

  // convert rotation matrix Re to a quaternion, q
  Eigen::Quaterniond q(Re);
  pose.position.x = Oe(0);
  pose.position.y = Oe(1);
  pose.position.z = Oe(2);

  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

// given a vector of joint states in DaVinci coords, convert these into
// equivalent DH parameters, theta and d
void Forward::convert_qvec_to_DH_vecs(const Vectorq7x1& q_vec)
{
  thetas_DH_vec_.resize(7);
  // +? -?
  thetas_DH_vec_ = theta_DH_offsets_;
  for (int i = 0; i < 2; i++)
  {
    thetas_DH_vec_(i)+= q_vec(i);
  }

  for (int i = 3; i < 7; i++)
  {
    thetas_DH_vec_(i)+= q_vec(i);
  }
  dvals_DH_vec_.resize(7);

  // +? -?
  dvals_DH_vec_ = dval_DH_offsets_;

  dvals_DH_vec_(2)+=q_vec(2);
}

double Forward::dh_var_to_qvec(double dh_val, int index)
{
  return (dh_val - DH_q_offsets[index]);
}

//    Eigen::VectorXd thetas_DH_vec_,dvals_DH_vec_;
Vectorq7x1 Forward::convert_DH_vecs_to_qvec(const Eigen::VectorXd &thetas_DH_vec,
  const Eigen::VectorXd &dvals_DH_vec)
{
  Vectorq7x1 q_vec;
  for (int i = 0; i < 7; i++)
  {
    q_vec(i) = thetas_DH_vec(i)-theta_DH_offsets_(i);
  }
  q_vec(2) = dvals_DH_vec(2)-dval_DH_offsets_(2);
  return q_vec;
}

// given 4 DH parameters, compute the corresponding transform as an affine3d
Eigen::Affine3d Forward::computeAffineOfDH(double a, double d, double alpha, double theta)
{
  Eigen::Affine3d affine_DH;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;

  // TODO(rcj) erdem IK bug #1: initialize 0 values in R matrix explicitly otherwise causes numerical issues
  double cq = cos(theta);
  double sq = sin(theta);
  double sa = sin(alpha);
  double ca = cos(alpha);
  R(0, 0) = cq;

  // - sin(q(i))*cos(alpha);
  R(0, 1) = -sq*ca;

  // sin(q(i))*sin(alpha);
  R(0, 2) = sq*sa;
  R(1, 0) = sq;
  // cos(q(i))*cos(alpha);
  R(1, 1) = cq*ca;
  R(1, 2) = -cq*sa;

  R(2, 0) = 0;
  R(2, 1) = sa;
  R(2, 2) = ca;
  affine_DH.linear() = R;

  p(0) = a * cq;
  p(1) = a * sq;
  p(2) = d;
  affine_DH.translation() = p;

  return affine_DH;
}

// use member fncs to compute and multiply successive transforms
Forward::Forward()
{
  // affine describing frame0 w/rt base frame--see comments above
  Eigen::Matrix3d R_0_wrt_base;
  Eigen::Vector3d Origin_0_wrt_base;
  Origin_0_wrt_base << 0, 0, 0;
  Eigen::Vector3d x_axis, y_axis, z_axis;
  // R_{0/base} = [0  1  0
  //               0  0 -1
  //              -1  0  0 ]
  // points IN, so + rotation is consistent leaning to the robot's left
  z_axis << 0, -1, 0;
  // choose x0 to point down, so will not have a joint-angle offset for pitch
  x_axis << 0, 0, -1;
  // consistent triad
  y_axis << 1, 0, 0;
  R_0_wrt_base.col(0) = x_axis;
  R_0_wrt_base.col(1) = y_axis;
  R_0_wrt_base.col(2) = z_axis;

  affine_frame0_wrt_base_.linear() = R_0_wrt_base;
  affine_frame0_wrt_base_.translation() = Origin_0_wrt_base;

  // fill in a static tool transform from frame6 to a frame of interest on the gripper
  affine_gripper_wrt_frame6_ = computeAffineOfDH(0, gripper_jaw_length, 0, -M_PI/2);

  theta_DH_offsets_.resize(7);
  for (int i = 0; i < 7; i++)
  {
    theta_DH_offsets_(i) = DH_q_offsets[i];
  }
  // don't put prismatic displacement here
  theta_DH_offsets_(2) = 0.0;

  dval_DH_offsets_.resize(7);
  dval_DH_offsets_<< 0, 0, DH_q_offsets[2], 0, 0, 0, 0;

  // resize MatrixXd Jacobian_ and initialize terms to 0's
  Jacobian_ = Eigen::MatrixXd::Zero(6, 6);
}

// provide DH theta and d values, return affine pose of gripper tip w/rt base frame
// also computes all intermediate affine frames, w/rt base frame
Eigen::Affine3d Forward::fwd_kin_solve_DH(const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& d_vec)
{
  // use or affect these member variables:
  affines_i_wrt_iminus1_.resize(7);
  Eigen::Affine3d xform;
  double a, d, theta, alpha;
  for (int i = 0; i < 7; i++)
  {
      a = DH_a_params[i];
      d = d_vec(i);
      alpha = DH_alpha_params[i];
      theta = theta_vec(i);
      xform = computeAffineOfDH(a, d, alpha, theta);
      affines_i_wrt_iminus1_[i]= xform;
  }

  affine_products_.resize(7);
  affine_products_[0] =  affine_frame0_wrt_base_ * affines_i_wrt_iminus1_[0];
  for (int i = 1; i < 7; i++)
  {
    affine_products_[i] = affine_products_[i-1] * affines_i_wrt_iminus1_[i];
  }
  affine_gripper_wrt_base_ = affine_products_[6] * affine_gripper_wrt_frame6_;
  return affine_gripper_wrt_base_;
}

Eigen::Affine3d Forward::fwd_kin_solve(const Vectorq7x1& q_vec)
{
  convert_qvec_to_DH_vecs(q_vec);
  Eigen::Affine3d forward_x_form(fwd_kin_solve_DH(thetas_DH_vec_, dvals_DH_vec_));

  return forward_x_form;
}

Eigen::Affine3d Forward::get_frame0_wrt_base() const
{
  return affine_frame0_wrt_base_;
}

Eigen::Affine3d Forward::get_gripper_wrt_frame6() const
{
  return affine_gripper_wrt_frame6_;
}

Eigen::MatrixXd Forward::compute_jacobian(const Vectorq7x1& q_vec)
{
  fwd_kin_solve(q_vec);
  Eigen::Vector3d z_axis;
  Eigen::Vector3d vec_tip_minus_Oi_wrt_base;
  Eigen::Matrix3d R;
  Eigen::Vector3d r_tip_wrt_base = affine_gripper_wrt_base_.translation();
  Eigen::Vector3d z_axis0 = affine_frame0_wrt_base_.linear().col(2);
  // angular Jacobian is just the z axes of each revolute joint (expressed in base frame);
  // for prismatic joint, there is no angular contribution
  // start from z_axis0

  // Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
  Jacobian_.block<3, 1>(3, 0) = z_axis0;
  vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - affine_frame0_wrt_base_.translation();
  Jacobian_.block<3, 1>(0, 0) = z_axis0.cross(vec_tip_minus_Oi_wrt_base);
  // 2nd joint:
  // refer to previous joint's z axis
  R = affine_products_[0].linear();
  z_axis = R.col(2);
  // Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
  Jacobian_.block<3, 1>(3, 1) = z_axis;
  vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - affine_products_[0].translation();
  Jacobian_.block<3, 1>(0, 1) = z_axis.cross(vec_tip_minus_Oi_wrt_base);

  // prismatic joint:
  R = affine_products_[1].linear();
  z_axis = R.col(2);
  Jacobian_.block<3, 1>(0, 2) = z_axis;

  // joints 4-6:
  for (int i = 3; i < 6; i++)
  {
    R = affine_products_[i-1].linear();
    z_axis = R.col(2);
    // Block of size (p,q), starting at (i,j) matrix.block<p,q>(i,j);
    Jacobian_.block<3, 1>(3, i) = z_axis;
    vec_tip_minus_Oi_wrt_base = r_tip_wrt_base - affine_products_[i-1].translation();
    Jacobian_.block<3, 1>(0, i) = z_axis.cross(vec_tip_minus_Oi_wrt_base);
  }
  // translational Jacobian depends on joint's z-axis and vector from i'th axis to robot tip
  return Jacobian_;
}

// gen_rand_legal_jnt_vals: compute random values within legal joint range:
void Forward::gen_rand_legal_jnt_vals(Vectorq7x1 &qvec)
{
  qvec(6) = 0;
  double drand_val;
  unsigned int seed = time(NULL);
  for (int i = 0; i < 6; i++)
  {
    drand_val = static_cast<double> (rand_r(&seed))/(static_cast<double> (RAND_MAX));
    qvec(i) = q_lower_limits[i] + (q_upper_limits[i] - q_lower_limits[i]) * drand_val;
  }
}

}  // namespace davinci_kinematics
