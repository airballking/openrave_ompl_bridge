cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/OMPLPlannersMain.cpp src/OMPLPlannerRRTConnect.cpp src/OMPLPlannerParametersRRTConnect.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME} ompl)

# UNIT TESTING
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
rosbuild_add_gtest(test/unit_tests test/UnitTests.cpp test/PluginTests.cpp test/RRTConnectParameters.cpp)
target_link_libraries(test/unit_tests ompl ${PROJECT_NAME})
