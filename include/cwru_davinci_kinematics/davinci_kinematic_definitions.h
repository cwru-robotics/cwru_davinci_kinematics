// @TODO Add License Text.
// Copyright Wyatt S. Newman and Russell Jackson 2017

#ifndef CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_H
#define CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_H

/**
 * @brief This list of constants are used for completing both the forward and inverse kinematic evaluation of a 
 * dvrk system.
 *
 * These measurements are also available from one of JHU's  dvrk website:
 * https://github.com/jhu-dvrk/sawIntuitiveResearchKit/blob/master/share/psm-large-needle-driver.json
 *
 * The kinematic models used in this library are based on DH parameters rather than joint location/direction as defined in the URDF standard.
 * However, the joint limits are the same as in the .xarco model files located in https://github.com/cwru-robotics/cwru_davinci_geometry_models.
 */
namespace davinci_kinematics
{

/**
 * @brief This distance is measured to be between the gripper jaw and the gripper wrist.
 */
extern const double dist_from_wrist_bend_axis_to_gripper_jaw_rot_axis;

/**
 * @brief This is the length of the gripper jaw. This is the part that actually grasps or cuts.
 */
extern const double gripper_jaw_length;

/**
 * @brief This is the list of Denavit–Hartenberg (DH) parameters which are often used for robot kinematic chains.
 * 
 * For further information about DH parameters, please consult https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
 */
extern const double DH_a_params[7];
extern const double DH_d_params[7];
extern const double DH_alpha_params[7];
extern const double DH_q_offsets[7];

/**
 * @brief These are the lower and upper limits of the dvrk Joints.
 */
extern const double q_lower_limits[7];
extern const double q_upper_limits[7];
}  // namespace davinci_kinematics

#endif  // CWRU_DAVINCI_KINEMATICS_DAVINCI_KINEMATIC_DEFINITIONS_H
