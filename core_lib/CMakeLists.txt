cmake_minimum_required(VERSION 3.8)
project(my_robot_core)

# 1. C++ 표준 설정 (C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. [핵심] 내 프로젝트 안의 Eigen 위치 지정
# (시스템 설치 경로가 아니라, 우리가 다운로드한 third_party 폴더를 가리킴)
set(EIGEN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen3)
include_directories(${EIGEN_PATH})

# 3. 소스 코드 추가 (테스트용 파일)
add_executable(core_test src/main.cpp)