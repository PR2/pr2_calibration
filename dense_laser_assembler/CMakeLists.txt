# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(dense_laser_assembler)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs calibration_msgs settlerlib roscpp_serialization)


include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})

add_library(dense_laser_assembler src/dense_laser_assembler.cpp)
target_link_libraries(dense_laser_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_serialization_LIBRARIES})
add_executable(dense_laser_imager src/dense_laser_imager.cpp)
target_link_libraries(dense_laser_imager ${roscpp_serialization_LIBRARIES} ${catkin_LIBRARIES})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)




## Generate added messages and services with any dependencies listed here
#generate_messages(
#    #TODO DEPENDENCIES geometry_msgs std_msgs
#)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp std_msgs sensor_msgs calibration_msgs settlerlib roscpp_serialization
    CATKIN_DEPENDS roscpp roscpp_serialization# TODO
    INCLUDE_DIRS include/${PROJECT_NAME}/ include # TODO include
    LIBRARIES dense_laser_assembler# TODO
)

catkin_add_gtest(test test/dense_laser_assembler_unittest.cpp)
target_link_libraries(test dense_laser_assembler ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS dense_laser_assembler
ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
