// @TODO Add License Text.
// Copyright Wyatt S. Newman 2015 and Russell Jackson 2017
// davinci_kinematics_test_main.cpp
// wsn, Sept 2015
// test function for davinci_kinematics library
// use as:
// FK:     affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec);
// IK:    q123 = ik_solver.q123_from_wrist(w_wrt_base);
// solves for theta1, theta2, d3 given wrist point (on wrist bend axis)

// zvec_4 from FK points along -yvec of tf frame one_tool_main_link (at tool rot=0)
// AND zvec_4 from FK points along y_vec of frame3 from FK at tool rot=0
// need an offset angle???

#include <davinci_kinematics/davinci_fwd_kinematics.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <sensor_msgs/JointState.h>
#include <string>

davinci_kinematics::Vectorq7x1 g_q_vec;  // global, filled by callback
bool g_got_callback = false;

void jsCallback(const sensor_msgs::JointState &jointState) {
	g_q_vec(0) = jointState.position[0];
	g_q_vec(1) = jointState.position[1];
	g_q_vec(2) = jointState.position[7];
	// rotation about tool shaft:
	g_q_vec(3) = jointState.position[8];
	// wrist bend:
	g_q_vec(4) = jointState.position[9];
	// q_vec[5] rotates both jaws together
	g_q_vec(5) = jointState.position[10];
	// decide what to do with this--2nd jaw
	g_q_vec(6) = 0.0;
	g_got_callback = true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "davinci_kinematics_test_main");

	ros::NodeHandle n;
	// create a Subscriber object and have it subscribe to the topic "topic1"
	// the function "myCallback" will wake up whenever a new message is published to topic1
	// the real work is done inside the callback function
	ros::Subscriber joint_state_subscriber = n.subscribe("dvrk_psm/joint_states", 1, jsCallback);
	davinci_kinematics::Forward davinci_fwd_solver;
	Eigen::Vector3d w_wrt_base, q123;

	ROS_INFO("waiting to receive joint states");
	while (!g_got_callback) {
		ros::spinOnce();
	}

	davinci_kinematics::Vectorq7x1 q_vec, err_vec, q_vec_ik;
	q_vec << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	// NEED THESE TO AGREE W/ JOINT_STATE_PUBLISHER
	/*
	 q_vec[0] = 0.0;
	 q_vec[1]= 0.0;
	 q_vec[2] = insertion_offset+0.0;
	 q_vec[3] = 0.0;
	 q_vec[4] = 0.0; // wrist bend
	 q_vec[5] = 0.0; //1.57; // gripper-jaw rotation
	 q_vec[6] = 0.0;
	 */
	// use published values:
	q_vec = g_q_vec;
	std::cout << "using q_vec = " << q_vec.transpose() << std::endl;

	tf::StampedTransform tf_wrist_wrt_base, tf_gripper_tip_wrt_base, tf_frame_wrt_base;
	tf::TransformListener tfListener;
	Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base, affine_frame_wrt_base;

	Eigen::Vector3d tip_from_FK, tip_from_FK_of_IK, tip_err;
	ROS_INFO("gripper tip frame from FK: ");
	affine_gripper_wrt_base = davinci_fwd_solver.fwd_kin_solve(q_vec);
	std::cout << "affine linear (R): " << std::endl;
	std::cout << affine_gripper_wrt_base.linear() << std::endl;
	tip_from_FK = affine_gripper_wrt_base.translation();
	std::cout << "origin: " << tip_from_FK.transpose() << std::endl;
	std::cout << std::endl;
	std::cout << std::endl;

	return 0;
}
