


#include <cwru_davinci_kinematics/davinci_kinematics.h>
#include <sensor_msgs/JointState.h>
#include <random>
// Bring in gtest
#include <gtest/gtest.h>

Vectorq7x1 randomJoint(std::default_random_engine generator)
{
	Vectorq7x1 newJoints;
	for (int i(0); i < 7; i++)
	{
		std::uniform_real_distribution<double> randU(q_lower_limits[i], q_upper_limits[i]);
		newJoints(i) = randU(generator);
	}
	return newJoints;
}

// Declare a test
TEST( davinci_kinematics, FullKinematics)
{
	// randomly generate a joint position:
	// compute the corresponding FK,
	// Compute the IK 
	// compare the new joints with the original joints.

	Davinci_fwd_solver FK_solver;
	Davinci_IK_solver IK_solver;
	std::default_random_engine generator;
	
	for (int i(0); i < 500; i++)
	{
		Vectorq7x1 q_vec(randomJoint(generator));
		Eigen::Affine3d newPose(FK_solver.fwd_kin_solve(q_vec));
		int solCount = IK_solver.ik_solve(newPose);
		EXPECT_GT(solCount, 0);

		Vectorq7x1 q_vecs = IK_solver.get_soln();
		double bestError = 10000000;
		double errorSum(0);
		for(int k(0); k < 7; k++)
		{
			double newErr = (q_vecs(k) - q_vec(k));
			errorSum += newErr*newErr;
		}
		if (errorSum < bestError)
		{
			bestError = errorSum;
		}
		if ((i % 100) == 0)
		{
			std::cout << q_vec << std::endl;
			printf("The sum squared error is: %f\n", bestError);
			std::cout << q_vecs << std::endl;
		}

		EXPECT_LT(bestError, 10);

	}
	SUCCEED();
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}