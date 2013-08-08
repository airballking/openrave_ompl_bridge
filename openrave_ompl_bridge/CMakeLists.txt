cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# BUILDING THE WRAPPED RRTCONNECT INTO A LIBRARY
rosbuild_add_library(${PROJECT_NAME} src/Robot.cpp src/RRTConnect.cpp src/RRTConnectParameters.cpp src/Conversions.cpp src/Environment.cpp) 
target_link_libraries(${PROJECT_NAME} ompl)
 
# BUILDING OUR CBiRRT PLANNER INTO A LIBRARY
rosbuild_add_library(CBIRRT src/CBiRRTSpace.cpp src/CBiRRTMotionValidator.cpp src/FeatureConstraints.cpp)
target_link_libraries(CBIRRT ompl)

# EXPOSING THE PLUGINS TO OPENRAVE IN A SEPARATE LIBRARY
rosbuild_add_library(OPENRAVE_PLUGINS src/PlannersMain.cpp)
target_link_libraries(OPENRAVE_PLUGINS ${PROJECT_NAME})

# UNIT TESTING
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
rosbuild_add_gtest(test/unit_tests test/UnitTests.cpp test/Robot.cpp test/RRTConnect.cpp test/RRTConnectParameters.cpp test/CBiRRTSpace.cpp test/CBiRRTSimple.cpp test/Conversions.cpp test/Environment.cpp test/FeatureConstraints.cpp)
target_link_libraries(test/unit_tests ${PROJECT_NAME} CBIRRT)
