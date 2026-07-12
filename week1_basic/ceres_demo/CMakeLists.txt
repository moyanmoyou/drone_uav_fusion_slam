cmake_minimum_required(VERSION 3.10)
project(ceres_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 找到Eigen和Ceres
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src)

# 任务1：基础曲线拟合
add_executable(1_curve_fitting src/1_curve_fitting.cpp)
target_link_libraries(1_curve_fitting ${CERES_LIBRARIES})

# 任务2：Huber鲁棒核抗外点拟合
add_executable(2_robust_fitting src/2_robust_fitting.cpp)
target_link_libraries(2_robust_fitting ${CERES_LIBRARIES})

add_executable(3_hand_eye_calib src/3_hand_eye_calib.cpp)
target_link_libraries(3_hand_eye_calib ${CERES_LIBRARIES})