# cwru_davinci_kinematics
[![Build Status](https://travis-ci.com/cwru-robotics/cwru_davinci_kinematics.svg?token=YmHMxBbcdppbMMkZWTut&branch=master)](https://travis-ci.com/cwru-robotics/cwru_davinci_kinematics)

CWRU's kinematic models for the IS 1200 (The original da Vinci Surgical Robot)

Added Jacobian computation.  Provides a 6x6 Jacobian.  Input is a 7x1 joint-space vector, identical to that used for
forward kinematics.  The 6x6 Jacobian ignores the jaw-opening angle (7'th joint of input vector).

To see how to use the Jacobian computation, see the test function, davinci_Jacobian_test_main.cpp, and comments therein.

