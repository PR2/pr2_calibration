# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

# CATKIN_MIGRATION: removed during catkin migration
cmake_minimum_required(VERSION 2.8.3)
project(pr2_calibration_launch)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS calibration_msgs calibration_estimation calibration_launch pr2_dense_laser_snapshotter laser_cb_detector laser_joint_processor laser_joint_projector stereo_image_proc image_cb_detector monocam_settler image_view robot_mechanism_controllers sensor_msgs geometry_msgs pr2_teleop rospy rostest kdl_parser)

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
#
# CATKIN_MIGRATION: removed during catkin migration
#  cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
#
# CATKIN_MIGRATION: removed during catkin migration
#  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
#
# CATKIN_MIGRATION: removed during catkin migration
#  rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})

#add_executable(pr2_fk_ref test/pr2_fk_ref.cpp)

#add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_pr2_fk.launch)

#uncomment if you have defined messages
#uncomment if you have defined services
add_service_files(
  FILES FkTest.srv
)
## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)
generate_messages()


## Generate added messages and services with any dependencies listed here

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS
    CATKIN_DEPENDS message_runtime# TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(
  DIRECTORY capture_data estimate_params
  results view_results intrinsics pr2_urdf_parser_py verify_urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)