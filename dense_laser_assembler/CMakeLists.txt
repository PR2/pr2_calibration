cmake_minimum_required(VERSION 2.8.3)
project(dense_laser_assembler)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs calibration_msgs settlerlib roscpp_serialization)


include_directories(include ${catkin_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})


#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

catkin_package(
    DEPENDS 
    CATKIN_DEPENDS roscpp roscpp_serialization calibration_msgs sensor_msgs settlerlib std_msgs# TODO
    INCLUDE_DIRS include/${PROJECT_NAME}/ include # TODO include
    LIBRARIES dense_laser_assembler
)

catkin_add_gtest(test test/dense_laser_assembler_unittest.cpp)
target_link_libraries(test dense_laser_assembler ${Boost_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


install(PROGRAMS scripts/assembler_node.py scripts/laser_image_node.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(dense_laser_assembler src/dense_laser_assembler.cpp)
target_link_libraries(dense_laser_assembler ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${roscpp_serialization_LIBRARIES})

add_executable(dense_laser_imager src/dense_laser_imager.cpp)
target_link_libraries(dense_laser_imager ${roscpp_serialization_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS dense_laser_assembler
ARCHIVE DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

