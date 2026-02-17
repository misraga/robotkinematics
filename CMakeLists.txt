cmake_minimum_required(VERSION 3.20)
project(robotkinematics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen (system install)
find_package(Eigen3 REQUIRED NO_MODULE)

# Library
add_library(robotkinematics src/kinematics.cpp)
target_include_directories(robotkinematics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robotkinematics PUBLIC Eigen3::Eigen)

# Example (optional)
option(BUILD_EXAMPLES "Build examples" ON)
if(BUILD_EXAMPLES)
  add_executable(rk_example examples/rk_example.cpp)
  target_link_libraries(rk_example PRIVATE robotkinematics)
endif()


