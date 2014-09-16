cmake_minimum_required(VERSION 2.8.3)
project(pr2_se_calibration_launch)
find_package(catkin REQUIRED COMPONENTS pr2_calibration_launch)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
    DEPENDS pr2_calibration_launch
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

install(DIRECTORY capture_data estimate_params write_urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES verify_pr2_se_uncalibrated.launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
