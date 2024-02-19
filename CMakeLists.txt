cmake_minimum_required(VERSION 3.10)

project(PlanarManipulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Ensure reslease build if not specified by user
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#  Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)

# Add source files
file(GLOB SOURCES "src/*.cpp")

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

#  Enable testing
enable_testing()
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
add_executable(manipulator_tests  tests/ManipulatorTest.cpp src/Manipulator.cpp src/Joint.cpp src/InverseKinematics.cpp)
add_executable(ik_tests  tests/InverseKinematicsTest.cpp src/Manipulator.cpp src/Joint.cpp src/InverseKinematics.cpp)

target_link_libraries(manipulator_tests  GTest::gtest GTest::gtest_main)
target_link_libraries(ik_tests GTest::gtest GTest::gtest_main)