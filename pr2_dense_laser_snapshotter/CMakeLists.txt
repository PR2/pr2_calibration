# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(pr2_dense_laser_snapshotter)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS dense_laser_assembler pr2_msgs)

include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
catkin_package(
    DEPENDS 
    CATKIN_DEPENDS pr2_msgs dense_laser_assembler# TODO
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

add_executable(dense_laser_snapshotter src/dense_laser_snapshotter.cpp)
target_link_libraries(dense_laser_snapshotter ${catkin_LIBRARIES})
add_dependencies(dense_laser_snapshotter ${catkin_EXPORTED_TARGETS}dense_laser_assembler_gencpp )

