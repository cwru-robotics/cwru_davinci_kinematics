cmake_minimum_required(VERSION 2.8.3)
project(cwru_davinci_kinematics)

#This is needed as part of the migration to ros jade and later
find_package(Eigen3)

# this is depreciated but still necessary for now.
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})
  set(EIGEN_DEP Eigen)
  # message("Found Eigen ${EIGEN_INCLUDE_DIR}")
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
  set(EIGEN_DEP Eigen3)
  # message("Found Eigen3 ${EIGEN3_INCLUDE_DIR}")
endif()

add_definitions(${EIGEN_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS roscpp roslint)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDES}
  ${EIGEN3_INCLUDE_DIRS}
)

catkin_package(
	DEPENDS ${EIGEN_DEP}
	LIBRARIES davinci_kinematics
	CATKIN_DEPENDS roscpp
	INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
)

# Libraries
add_library(davinci_kinematics src/davinci_kinematics.cpp)
target_link_libraries(davinci_kinematics ${catkin_LIBRARIES})


#Ros linting:

roslint_cpp()

# Test lists:

roslint_add_test()

# TODO(rcj), convert these tests into unit testing for code verification.

#cs_add_library(davinci_joint_publisher src/davinci_joint_publisher.cpp) 
# Executables
#cs_add_executable(test_davinci_ik src/davinci_kinematics_test_main2.cpp)
#cs_add_executable(test_davinci_kin3 src/davinci_kinematics_test_main3.cpp)
#cs_add_executable(test_davinci_kin4 src/davinci_kinematics_test_main4.cpp)
#cs_add_executable(test_davinci_joint_pub src/davinci_joint_publisher_test_main.cpp)
#cs_add_executable(test_pointer src/test_pointer.cpp)
#cs_add_executable(davinci_cart_traj_pub src/davinci_cart_traj_pub.cpp)
#cs_add_executable(davinci_playfile_jointspace src/davinci_playfile_jointspace.cpp)
#target_link_libraries(test_davinci_ik davinci_kinematics ${catkin_LIBRARIES})
#target_link_libraries(test_davinci_kin3 davinci_kinematics ${catkin_LIBRARIES})
#target_link_libraries(test_davinci_kin4 davinci_kinematics ${catkin_LIBRARIES})
#target_link_libraries(test_davinci_joint_pub davinci_joint_publisher ${catkin_LIBRARIES})
#target_link_libraries(davinci_cart_traj_pub davinci_joint_publisher davinci_kinematics ${catkin_LIBRARIES})
#target_link_libraries(davinci_playfile_jointspace davinci_joint_publisher davinci_kinematics ${catkin_LIBRARIES})
#target_link_libraries(test_pointer davinci_joint_publisher davinci_kinematics ${catkin_LIBRARIES})
