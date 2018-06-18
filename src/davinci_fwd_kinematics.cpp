/*
 *  davinci_fwd_kinematics.cpp
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

// given a vector of joint states in DaVinci coords, convert these into
// equivalent DH parameters, theta and d
void Forward::convert_qvec_to_DH_vecs(const Vectorq7x1& q_vec, Eigen::VectorXd &thetas_DH_vec,
  Eigen::VectorXd &dvals_DH_vec)
{
  thetas_DH_vec.resize(7);
  // +? -?
  thetas_DH_vec = theta_DH_offsets_;
  for (int i = 0; i < 7; i++)
  {
    // skip the linear joint.
    if (i == 2) continue;
    thetas_DH_vec(i)+= q_vec(i);
  }

  dvals_DH_vec.resize(7);
  dvals_DH_vec = dval_DH_offsets_;
  dvals_DH_vec(2) += q_vec(2); // RN original
	// dvals_DH_vec(2) += 0.988*q_vec(2); // RN 20180220  (TODO adjust DH_a1 at the same time!)
  //dvals_DH_vec(1) += q_vec(2);

	// dvals_DH_vec(2) += 0.98*q_vec(2); // RN 20180319A2


}

int Forward::check_jnts(const Vectorq7x1& q_vec)
{
  int result(0);
  for (int i(0); i < 6; i++)
  {
    if (q_vec(i) < q_lower_limits[i] || q_vec(i) > q_upper_limits[i])
    {
      result -= (1 << i);
    }
  }
  if (result < 0)
  {
    return result;
  }
  // The wrist is inside the cannula.
  if (q_vec(2) < cannula_short_length)
  {
    // if the wrist is straight, then it is ok.
    for (unsigned int i(3); i < 6; i++)
    {
      if (abs(q_vec(i)) > 0.001)
      {
        return 1;
      }
    }
  }
  // Test if the gripper is open wider than available.
  if (abs(q_vec(6)) > (M_PI -abs(q_vec(5))))
  {
    return 2;
  }
  return 0;
}

double Forward::dh_var_to_qvec(double dh_val, int index)
{
  return (dh_val - DH_q_offsets[index]);
}



// RN TODO deal with that .99 sacle factor
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
  set_gripper_jaw_length(gripper_jaw_length);

  theta_DH_offsets_.resize(7);
  for (int i = 0; i < 7; i++)
  {
    theta_DH_offsets_(i) = DH_q_offsets[i];

    // std::cout << "RNDEBUG" << std::endl << "theta_DH_offsets_(i): " << theta_DH_offsets_(i)  <<std::endl;
  }
  // don't put prismatic displacement here
  // theta_DH_offsets_(2) = 0.0;
  theta_DH_offsets_(2) = 0.0;
  // std::cout << "RNDEBUG" << std::endl << "theta_DH_offsets_(2): " << theta_DH_offsets_(2)  <<std::endl;

  dval_DH_offsets_.resize(7);
	// dval_DH_offsets_<< 0, 0 , DH_q_offsets[2], 0, 0, 0, 0;
  // dval_DH_offsets_<< 0, -0.001 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180214
  // dval_DH_offsets_<< 0, 0.0013 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180215A1
  // dval_DH_offsets_<< 0, -0.002 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180216
	// dval_DH_offsets_<< 0, -0.0034 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180218A1
	// dval_DH_offsets_<< 0, -0.002784 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180218A2
  // dval_DH_offsets_<< 0, -0.002986 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180219A1
  // dval_DH_offsets_<< 0, -0.0030 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180222A1 GOOD
	// dval_DH_offsets_<< 0, -0.0031 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180319 GOOD <<

// dval_DH_offsets_<< 0, -0.001 , DH_q_offsets[2], 0, 0, 0, 0; //RN 20180319PSM2A1
// dval_DH_offsets_<< 0, -0.00074152, DH_q_offsets[2], 0, 0, 0, 0; // RN 20180618A1 PSM2
dval_DH_offsets_<< 0,  -0.0012186, DH_q_offsets[2], 0, 0, 0, 0; // RN 20180618A1 PSM2

  // resize MatrixXd Jacobian_ and initialize terms to 0's
  Jacobian_ = Eigen::MatrixXd::Zero(6, 6);
}

// provide DH theta and d values, return affine pose of gripper tip w/rt base frame
// also computes all intermediate affine frames, w/rt base frame
void Forward::fwd_kin_solve_DH(const Eigen::VectorXd& theta_vec, const Eigen::VectorXd& d_vec)
{
  // use or affect these member variables:

  std::vector <Eigen::Affine3d> affines_i_wrt_iminus1;
  affines_i_wrt_iminus1.resize(7);
  Eigen::Affine3d xform;
  double a, d, theta, alpha;
  for (int i = 0; i < 7; i++)
  {
    a = DH_a_params[i];
    d = d_vec(i);
    alpha = DH_alpha_params[i];
    theta = theta_vec(i);
    xform = computeAffineOfDH(a, d, alpha, theta);
    affines_i_wrt_iminus1[i]= xform;
  }
  affine_products_.resize(7);
  affine_products_[0] =  affine_frame0_wrt_base_ * affines_i_wrt_iminus1[0];
  for (int i = 1; i < 7; i++)
  {
    affine_products_[i] = affine_products_[i-1] * affines_i_wrt_iminus1[i];
  }
  affine_gripper_wrt_base_ = affine_products_[6] * affine_gripper_wrt_frame6_;

  // RN added for wrist pt coordinate w/rt base frame
  affine_wrist_wrt_base_ = affine_products_[2];
}

Eigen::Affine3d Forward::get_wrist_wrt_base() // RN
{
  return affine_wrist_wrt_base_;
}

Eigen::Affine3d Forward::fwd_kin_solve(const Vectorq7x1& q_vec)
{
  current_joint_state_ = q_vec;
  Eigen::VectorXd thetas_DH_vec, dvals_DH_vec;
  convert_qvec_to_DH_vecs(q_vec, thetas_DH_vec, dvals_DH_vec);
  fwd_kin_solve_DH(thetas_DH_vec, dvals_DH_vec);

  return affine_gripper_wrt_base_;
}

Eigen::Affine3d Forward::fwd_kin_solve()
{
  return affine_gripper_wrt_base_;
}

Eigen::Affine3d Forward::get_frame0_wrt_base() const
{
  return affine_frame0_wrt_base_;
}

void Forward::set_frame0_wrt_base(const Eigen::Affine3d &affine_frame0_wrt_base)
{
  affine_frame0_wrt_base_ = affine_frame0_wrt_base;
}

Eigen::Affine3d Forward::get_gripper_wrt_frame6() const
{
  return affine_gripper_wrt_frame6_;
}

void Forward::set_gripper_jaw_length(double  jaw_length)
{
  this->gripper_jaw_length_ = jaw_length;
  affine_gripper_wrt_frame6_ = computeAffineOfDH(0, gripper_jaw_length, 0, -M_PI/2);
}

Eigen::MatrixXd Forward::compute_jacobian(const Vectorq7x1& q_vec)
{
  // use the jacobian to make the computation.
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

Eigen::MatrixXd Forward::compute_jacobian()
{
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
