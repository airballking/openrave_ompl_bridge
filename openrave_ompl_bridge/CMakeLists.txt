cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(${PROJECT_NAME} src/Robot.cpp src/OMPLPlannerRRTConnect.cpp src/OMPLPlannerParametersRRTConnect.cpp) 
target_link_libraries(${PROJECT_NAME} ompl)

#rosbuild_add_library(${PROJECT_NAME} src/OMPLPlannersMain.cpp src/OMPLPlannerRRTConnect.cpp src/OMPLPlannerParametersRRTConnect.cpp)

#rosbuild_add_library(NO_PLUGIN src/OMPLPlannerRRTConnect.cpp src/OMPLPlannerParametersRRTConnect.cpp)

# UNIT TESTING
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
rosbuild_add_gtest(test/unit_tests test/UnitTests.cpp test/Robot.cpp)
target_link_libraries(test/unit_tests ${PROJECT_NAME})

#rosbuild_add_gtest(test/unit_tests test/UnitTests.cpp test/PluginTests.cpp test/RRTConnectParameters.cpp test/RRTConnectPlanner.cpp)
#target_link_libraries(test/unit_tests ompl NO_PLUGIN)
