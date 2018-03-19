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

	Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base;

	davinci_kinematics::Vectorq7x1 q_vec, q_vec_up, q_vec_bottom;
	// 0.0,	0.0,	0.05,	-0.9,	1.5,	0,	0.139,
  //	q_vec_up(0) = 0;
  //	q_vec_up(1) = 0;
  //  q_vec_up(2) = 0.05;
  //	q_vec_up(3) = -0.9;
  //	q_vec_up(4) = 1.5;
  //	q_vec_up(5) = 0;
  //	q_vec_up(6) = 0;

	// 0.0,	0.0,	0.21,	0,	0,	0,	0.139,		0,0,0,0,0,0,0,	10
  //	q_vec_bottom(0) = 0;
  //	q_vec_bottom(1) = 0;
  //  q_vec_bottom(2) = 0.21;
  //	q_vec_bottom(3) = -0.9;
  //	q_vec_bottom(4) = 1.5;
  //	q_vec_bottom(5) = 0;
  //	q_vec_bottom(6) = 0;
		
  //	affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec_up);

  //	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();

  //  std::cout << "q_vec_up" << std::endl;
  //	std::cout << q_vec_up.transpose() << std::endl << std::endl;

  //  std::cout << "q_vec_up.affine_wrist_wrt_base.linear():" << std::endl;
  //  std::cout << affine_wrist_wrt_base.linear() << std::endl << std::endl;
  //  std::cout << "q_vec_up.affine_wrist_wrt_base.translation():" << std::endl;
  //  std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;

  //  std::cout << "q_vec_up.affine_gripper_wrt_base.linear():" << std::endl;
  //  std::cout << affine_gripper_wrt_base.linear() << std::endl << std::endl;
  //  std::cout << "q_vec_up.affine_gripper_wrt_base.translation():" << std::endl;
  //  std::cout << affine_gripper_wrt_base.translation() << std::endl << std::endl;

  //	affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec_bottom);

  //	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();

  //  std::cout << "q_vec_bottom" << std::endl;
  //	std::cout << q_vec_bottom.transpose() << std::endl << std::endl;

  //  std::cout << "q_vec_bottom.affine_wrist_wrt_base.linear():" << std::endl;
  //  std::cout << affine_wrist_wrt_base.linear() << std::endl << std::endl;
  //  std::cout << "q_vec_bottom.affine_wrist_wrt_base.translation():" << std::endl;
  //  std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;

  //  std::cout << "q_vec_bottom.affine_gripper_wrt_base.linear():" << std::endl;
  //  std::cout << affine_gripper_wrt_base.linear() << std::endl << std::endl;
  //  std::cout << "q_vec_bottom.affine_gripper_wrt_base.translation():" << std::endl;
  // std::cout << affine_gripper_wrt_base.translation() << std::endl << std::endl;

	q_vec(0) = 0;
	q_vec(1) = 0;
  q_vec(2) = 0.05;
	q_vec(3) = 0;
	q_vec(4) = 0;
	q_vec(5) = 0;
	q_vec(6) = 0;

	for (int i = 0; i < 4; i++)
	{
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
		std::cout << "-----------------------------------------" << std::endl;
  	std::cout << "q_vec#" << i+1 << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << i+1 << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
		// std::cout << "q_vec#" << i+1 <<".affine_wrist_wrt_base.linear():" << std::endl;
		// std::cout << affine_wrist_wrt_base.linear() << std::endl << std::endl;

		std::cout << "-----------------------------------------" << std::endl;

		q_vec(2) = q_vec(2) + 0.06;
	

	}

	std::cout << "----------------RANDOM-POINTS--------------" << std::endl;

	std::cout << "-------PT01---------" << std::endl;
	q_vec(0) = -1.0800;
	q_vec(1) = -0.6600;
  q_vec(2) =  0.0842;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT02---------" << std::endl;
	q_vec(0) = 1.3500;
	q_vec(1) = 0.3000;
  q_vec(2) = 0.1166;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT03---------" << std::endl;
	q_vec(0) = 1.3800;
	q_vec(1) = 0.4800;
  q_vec(2) = 0.1634;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT04---------" << std::endl;
	q_vec(0) = 0.2400;
	q_vec(1) = 0.3000;
  q_vec(2) = 0.1922;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT05---------" << std::endl;
	q_vec(0) = -1.3200;
	q_vec(1) = -0.0800;
  q_vec(2) =  0.0662;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT06---------" << std::endl;
	q_vec(0) = -0.7800;
	q_vec(1) =  0.1000;
  q_vec(2) =  0.2174;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT07---------" << std::endl;
	q_vec(0) = -0.4200;
	q_vec(1) = -0.4000;
  q_vec(2) =  0.1904;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT08---------" << std::endl;
	q_vec(0) = 0.9900;
	q_vec(1) = 0.5000;
  q_vec(2) = 0.1382;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT09---------" << std::endl;
	q_vec(0) = -1.4400;
	q_vec(1) = -0.6200;
  q_vec(2) =  0.1292;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;

	std::cout << "-------PT10---------" << std::endl;
	q_vec(0) = -1.3500;
	q_vec(1) =  0.3800;
  q_vec(2) =  0.1310;
		affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);
  	affine_wrist_wrt_base = dvrk_forward.get_wrist_wrt_base();
  	std::cout << "q_vec#" << ": " << std::endl;
		std::cout << q_vec.transpose() << std::endl << std::endl;

	  std::cout << "q_vec#" << ".affine_wrist_wrt_base.translation():" << std::endl;
  	std::cout << affine_wrist_wrt_base.translation() << std::endl << std::endl;
	std::cout << "--------------------" << std::endl;









} // main
