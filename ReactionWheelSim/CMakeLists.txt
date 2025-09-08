# ReactionWheelSim/CMakeLists.txt
cmake_minimum_required(VERSION 3.16)
project(rw_commander_cpp LANGUAGES CXX)

# Only build this one tool
add_executable(rw_commanderCPP rw_commanderCPP.cpp)

# C++ standard + warnings
target_compile_features(rw_commanderCPP PRIVATE cxx_std_17)
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(rw_commanderCPP PRIVATE -O2 -Wall -Wextra -Wpedantic)
endif()
