

#ifndef DAVINCI_KIN_PRE_H
#define	DAVINCI_KIN_PRE_H

#include <vector>
#include <string>

using namespace std;

const double dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis = 0.0091;
const double gripper_jaw_length = 0.0102; 

//What the hell is 'DH'?
const double DH_a1=0.0; //origin0 coincident w/ origin1
const double DH_a2=0.0; // axis z1,z2 intersect
const double DH_a3=0.0; //axes z2 (prismatic) and z3 (shaft rot) intersect
const double DH_a4=0.0; //axes z3 (shaft rot) and z4 (wrist bend) intersect
// axes z4 (wrist bend) and z5 (gripper-jaw rot axis) do NOT intersect:
const double DH_a5=dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis;  
const double DH_a6=0.0; // define tool frame on jaw w/ z6 axis intersecting z5 axis

const double DH_a7=0.0; //not sure what to do with this one


const double DH_d1 = 0.0;// 
const double DH_d2 = 0.0;//  THIS IS VARIABLE
const double DH_d3 = 0.0; 
const double DH_d4 = 0.0;
const double DH_d5 = 0.0; 
const double DH_d6 = 0.0;
const double DH_d7 = 0.0; 

//robot.DH.alpha= '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
const double DH_alpha1 = M_PI/2.0;
const double DH_alpha2 = M_PI/2.0;
const double DH_alpha3 = 0.0; // prismatic axis is aligned with tool-shaft spin axis 
const double DH_alpha4 = M_PI/2.0;
const double DH_alpha5 = -M_PI/2.0; // offset from wrist bend to jaw bend axis--> alpha is defined -pi/2
const double DH_alpha6 = M_PI/2.0; // choose gripper frame to have z-axis pointing along jaws
const double DH_alpha7 = 0.0; // relevant only if treat jaws as separate links...


// q_Davinci vec: starts counting from 0; 0 displacement at model "home"
//  when model joint_states are all at displacement 0.0
// use these offsets to convert to DH coords

const double insertion_offset=0.0156; //must command this much displacement to get wrist-bend axis to intersect base origin

const double DH_q_offset0 = 0.0;
const double DH_q_offset1 = M_PI/2.0;
const double DH_q_offset2 = -insertion_offset; // erdem IK tested, sign is negative. q3 should be larger than abs(insertion_offset) 
const double DH_q_offset3 = M_PI;
const double DH_q_offset4 = M_PI/2;
const double DH_q_offset5 = M_PI/2; //M_PI;
const double DH_q_offset6 = 0.0; //M_PI;

const double deg2rad = M_PI/180.0;

// NEED TO FIND THESE:
const double DH_q_max0 = 1.0; //deg2rad*45; //141; //51;
const double DH_q_max1 = 0.7; //deg2rad*45;
const double DH_q_max2 = 0.23; //0.5;
const double DH_q_max3 = 2.25; //deg2rad*180;
const double DH_q_max4 = 1.57; //deg2rad*90;
const double DH_q_max5 = 1.39; //deg2rad*90; //
const double DH_q_max6 = 1.57; //deg2rad*90;

//-141, -123, -173.5, -3, -175.25, -90, -175.25
const double DH_q_min0 = -1.0; //-deg2rad*45; //51; //141;
const double DH_q_min1 = -0.7; //-deg2rad*45;
const double DH_q_min2 =  0.01; 
const double DH_q_min3 = -2.25; //-deg2rad*180;
const double DH_q_min4 = -1.57; //-deg2rad*90;
const double DH_q_min5 = -1.39; //-deg2rad*90; //
const double DH_q_min6 = -1.57; //-deg2rad*90;

const double DH_a_params[7]={DH_a1,DH_a2,DH_a3,DH_a4,DH_a5,DH_a6,DH_a7};
double DH_d_params[7] = {DH_d1, DH_d2, DH_d3, DH_d4, DH_d5, DH_d6,DH_d7};
const double DH_alpha_params[7] = {DH_alpha1, DH_alpha2, DH_alpha3, DH_alpha4, DH_alpha5, DH_alpha6,DH_alpha7};
const double DH_q_offsets[7] = {DH_q_offset0,DH_q_offset1, DH_q_offset2, DH_q_offset3, DH_q_offset4, DH_q_offset5, DH_q_offset6};
const double q_lower_limits[7] = {DH_q_min0,DH_q_min1, DH_q_min2, DH_q_min3, DH_q_min4, DH_q_min5, DH_q_min6};
const double q_upper_limits[7] = {DH_q_max0,DH_q_max1, DH_q_max2, DH_q_max3, DH_q_max4, DH_q_max5, DH_q_max6};


//frames: 
 //world->
 //   one_psm_base_link->
 //     one_outer_yaw_link-->
 //        one_outer_pitch_link-->
 //          one_tool_main_link-->
 //            one_tool_wrist_link-->
 //               one_tool_wrist_shaft_link-->
 //                  one_tool_wrist_sca_link--> z-axis through wrist bend axis, but moves w/ wrist bend
 //                     one_tool_wrist_sca_shaft_link-->: z-axis thru gripper jaw rot axis
 //                       one_tool_tip_link

//I don't think anything uses these anymore.
string q1_psm1_jnt_name("one_outer_yaw_joint"); //cradle tip left/right
string q2_psm1_jnt_name("one_outer_pitch_joint"); //4-bar linkage, lean fwd/back
string d3_psm1_jnt_name("one_outer_insertion_joint"); //shaft insertion, prismatic jnt
string q4_psm1_jnt_name("one_outer_roll_joint"); //shaft rotation
string q5_psm1_jnt_name("one_outer_wrist_pitch_joint"); // wrist bend
string q6_psm1_jnt_name("one_outer_wrist_yaw_joint"); // jaw rotation?
string q7_psm1_jnt_name("one_outer_wrist_open_angle_joint"); // gripper opening?

string q1_psm2_jnt_name("two_outer_yaw_joint"); //cradle tip left/right
string q2_psm2_jnt_name("two_outer_pitch_joint"); //4-bar linkage, lean fwd/back
string d3_psm2_jnt_name("two_outer_insertion_joint"); //shaft insertion, prismatic jnt
string q4_psm2_jnt_name("two_outer_roll_joint"); //shaft rotation
string q5_psm2_jnt_name("two_outer_wrist_pitch_joint"); // wrist bend
string q6_psm2_jnt_name("two_outer_wrist_yaw_joint"); // jaw rotation?
string q7_psm2_jnt_name("two_outer_wrist_open_angle_joint"); // gripper opening?



#endif