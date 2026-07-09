cmake_minimum_required(VERSION 3.10)
project(eigen_imu_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)

file(GLOB SOURCES src/*.cpp)

add_executable(eigen_imu_test ${SOURCES})