# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(laser_joint_processor)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS geometry_msgs message_filters settlerlib joint_states_settler)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS signals thread)



include_directories(${Boost_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type. Options are:
# Coverage : w/ debug symbols, w/o optimization, w/ code-coverage
# Debug : w/ debug symbols, w/o optimization
# Release : w/o debug symbols, w/ optimization
# RelWithDebInfo : w/ debug symbols, w/ optimization
# MinSizeRel : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#find_package(OpenCV)
#find_package(OpenCV2)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()



#common commands for building c++ executables and libraries
#add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

## Generate added messages and services with any dependencies listed here
#generate_messages(
# #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS geometry_msgs message_filters settlerlib joint_states_settler opencv2
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include/${PROJECT_NAME}# TODO include
    LIBRARIES # TODO
)

add_library(${PROJECT_NAME} src/joint_imager.cpp
                                     src/laser_joint_processor.cpp
                                     src/joint_image_interpolator.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

add_executable(laser_joint_processor_node src/laser_joint_processor_node.cpp)
target_link_libraries(laser_joint_processor_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


target_link_libraries(laser_joint_processor_node ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})

#catkin_add_gtest(joint_imager_unittest test/joint_imager_unittest.cpp)
#target_link_libraries(joint_imager_unittest laser_joint_processor)

#catkin_add_gtest(joint_image_interpolator_unittest test/joint_image_interpolator_unittest.cpp)
#target_link_libraries(joint_image_interpolator_unittest laser_joint_processor)

#catkin_add_gtest(interp_snapshot_unittest test/interp_snapshot_unittest.cpp)
#target_link_libraries(interp_snapshot_unittest laser_joint_processor)

#catkin_add_gtest(laser_joint_processor_unittest test/#laser_joint_processor_unittest.cpp)
#target_link_libraries(laser_joint_processor_unittest laser_joint_processor)

