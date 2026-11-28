# Registers ../smoke.launch.py with colcon test. Copy this folder into a workspace of your own (see README step 4);
# COLCON_IGNORE keeps the course's own colcon build from picking it up here.
cmake_minimum_required(VERSION 3.8)
project(p14_smoke)
find_package(ament_cmake REQUIRED)
if(NOT DEFINED ENV{SMOKE_TEST})
  message(FATAL_ERROR "set SMOKE_TEST to the absolute path of projects/p14_test-the-stack/smoke.launch.py")
endif()
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test($ENV{SMOKE_TEST} TIMEOUT 300)
  add_launch_test($ENV{SMOKE_TEST} TARGET smoke_no_estimation ARGS "estimation:=false" TIMEOUT 300)
endif()
ament_package()
