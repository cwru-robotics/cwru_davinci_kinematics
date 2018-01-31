#include <cwru_davinci_kinematics/davinci_fwd_kinematics.h>
#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>
#include <Eigen/Geometry>

int main(int argc, char **argv){
	Eigen::Vector3d x_vec = Eigen::Vector3d(0.0, 0.0,  1.0);
	Eigen::Vector3d z_vec = Eigen::Vector3d(1.0, 0.0,  0.0);
	Eigen::Vector3d t_vec = Eigen::Vector3d(0.0, 0.0, -0.1);
	
	Eigen::Vector3d y_vec = z_vec.cross(x_vec);
	Eigen::Affine3d des_gripper_affine;
	Eigen::Matrix3d R;
	R.col(0) = x_vec;
	R.col(1) = y_vec;
	R.col(2) = z_vec;
	des_gripper_affine.linear() = R;
	des_gripper_affine.translation() = t_vec;
	
	std::cout << "What came in was\n" << des_gripper_affine.linear() << "\n\n" << des_gripper_affine.translation() << "\n\n";
	
	davinci_kinematics::Inverse kin = davinci_kinematics::Inverse();
	int s = kin.ik_solve(des_gripper_affine);
	if(s < 1){
		std::cout << "Inverse kinematics is rejecting.\n\n";
	} else{
		std::cout << "Inverse kinematics is OK.\n\n";
	}
	
	davinci_kinematics::Vectorq7x1 q_vec1 = kin.get_soln();
	
	std::cout << "Joint vector\n" << q_vec1 << "\n\n";
	
	davinci_kinematics::Forward fkin = davinci_kinematics::Forward();
	Eigen::Affine3d output = fkin.fwd_kin_solve(q_vec1);
	
	std::cout << "What came out was\n" << output.linear() << "\n\n" << output.translation() << "\n\n";
	
	s = kin.ik_solve(des_gripper_affine);
	if(s < 1){
		std::cout << "Inverse kinematics is rejecting.\n\n";
	} else{
		std::cout << "Inverse kinematics is OK.\n\n";
	}
	
	q_vec1 = kin.get_soln();
	
	std::cout << "Joint vector, again\n" << q_vec1 << "\n\n";

	return 0;
}
