# cwru_davinci_kinematics
[![Build Status](https://travis-ci.org/cwru-robotics/cwru_davinci_kinematics.svg?branch=master)](https://travis-ci.org/cwru-robotics/cwru_davinci_kinematics)

CWRU's kinematic models for the IS 1200 (The original da Vinci Surgical Robot)

Added Jacobian computation.  Provides a 6x6 Jacobian.  Input is a 7x1 joint-space vector, identical to that used for
forward kinematics.  The 6x6 Jacobian ignores the jaw-opening angle (7'th joint of input vector).

To see how to use the Jacobian computation, see the test function, davinci_Jacobian_test_main.cpp, and comments therein.

## Kinematics Outline:

The kinematics explanation was originally copied from the kinematics header file.
### TODO organize the explanation and add figures as necessary.

define D-H frames;
 "base" frame is same as "one_psm_base_link"
  O_base is at pivot (trocar) point
  z-axis points "up", y-axis points forward, x-axis is to robot's right

 DH-0 frame: this is a static transform from base frame, but conforms to
 DH convention that z0 is through first joint axis.  
 get freedom to choose orientation of x0, y0 spin about z0
 O_0 = O_base
 + rotation of q[0]==q1 corresponds to "leaning to the left", -->
 z_0 points along -y axis of base frame;

 choose x0 pointing "down", so yaw home will agree with davinci q[0]
 then y0 points along +x_base
 R_{0/base} = [0  1  0
               0  0 -1
              -1  0  0 ]

 DH-1 frame:  this frame moves w/ yaw joint (theta1 or q(0))
 construct from z_0 crossed into z_1==> x_1
 z_1 axis points to left (positive rotation of q[1]==q2 ==> pitch "leans forward")
 + rotation is about z-axis pointing to the left, i.e. coincident w/ -x0 (in home position)
 cross z_0 into z_1 ==> x1 axis points "down" in home position
 by construction, alpha1 = +pi/2.
 O_1 = O_0 ==> a1=0, d1=0
 --> need a psi_0 offset = +/- pi/2 so davinci home (in psi coords) has tool shaft parallel to z_base
 psi[0] = theta1_DH- pi/2


 DH-2 frame: construct from z1 (pitch axis) and z2 (prismatic axis)
 choose prismatic axis pointing through tool-shaft centerline, towards gripper
 choose origin O_2 to lie on z1 at intersection w/ wrist-bend joint
 define d2=0 such that O_2 is at O_base = O_1 = O_2
 from home pose, z1 is to the left, and z2 is down, so x2 points inwards, towards robot
 but x1 axis points "down" at Davinci home pose;
 this corresponds to a theta3 of +pi/2...and this value is static;
 by construction, alpha_2 = +pi/2
 a2=0, 
 d2 is a variable (prismatic joint displacement)
 

 DH-3 frame: z axis is spin about tool shaft, coincident w/ displacement axis z3
 for + rotation, z3 points same direction as z2, so alpha3 = 0
 choose origin coincident: O_3 = O_2, so a3 = d3 = 0
 get freedom to choose frame orientation of x3 (perpendicular to z3)
 define x3 direction such that thetaDH4 = q[3] (no offset correction needed; in agreement at home pose)
 at home pose, x3 points towards robot--coincident w/ x2 of prismatic jnt


 DH-4 frame: construct from tool-shaft spin axis, z3 and wrist-bend z4 axes
 by choice of origin for O3, have O_3 = O_4
 --> a4=0, d4=0
 by construction, x4 = z3 crossed into z4 and alpha4= +pi/2
 for plus wrist bend, at home pose, z4 points to "left"
 so, z3 cross z4 = x4 points FORWARD
 this is pi away from x3, so need offset such that q[3]=0 when thetaDH_4 = pi


 DH-5 frame: z5 is through gripper-jaw rotation axis
 at home pose, z5 points "IN"...s.t. + rotation causes gripper jaws to point to right
 O_5 is on the z5 axis, offset from O_4
 z4 and z5 do not intersect.  Have a non-zero a5 offset
 min dist from z4 to z5 defines x5; points from O4 to O_5
 +rotation direction of jaw rotation--> alpha5 is - pi/2
 d5 = 0
 need to find theta5 (wrist-bend) offset for Davinci home

 DH-6 frame: choose a final gripper frame (don't have two axes to construct)
 try z6 pointing out from z5 axis at +/- pi/2 = alpha6;
 set coincident origin, a6 = d6 = 0; O_6 = O_5 (then use separate transform to gripper tip, if desired)
 find theta6 offset to conform w/ Davinci home


