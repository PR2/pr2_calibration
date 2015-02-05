# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(laser_joint_projector)
find_package(catkin REQUIRED COMPONENTS calibration_msgs roscpp geometry_msgs roscpp_serialization)
find_package(cmake_modules REQUIRED)

find_package(kdl_parser REQUIRED)
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)
include_directories(SYSTEM ${Eigen_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${kdl_parser_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})
#link_directories(${PROJECT_NAME} ${catkin_INCLUDE_DIRS})

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()
#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)




#uncomment if you have defined messages

#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
## Generate added messages and services with any dependencies listed here

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS calibration_msgs kdl_parser geometry_msgs roscpp_serialization laser_joint_projector
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)
add_library(${PROJECT_NAME} src/laser_joint_projector.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(laser_joint_projector_node src/laser_joint_projector_node.cpp)
target_link_libraries(laser_joint_projector_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(keyboard_float_generator src/keyboard_float_generator.cpp)
target_link_libraries(keyboard_float_generator ${catkin_LIBRARIES})

add_executable(time_alignment_node src/time_alignment_node.cpp)
target_link_libraries(time_alignment_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen_LIBRARIES} ${kdl_parser_LIBRARIES} ${orocos_kdl_LIBRARIES})


