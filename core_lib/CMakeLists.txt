cmake_minimum_required(VERSION 3.8)
project(my_robot_core)

# 1. C++ 표준 설정 (C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. [핵심] 내 프로젝트 안의 Eigen 위치 지정
# (시스템 설치 경로가 아니라, 우리가 다운로드한 third_party 폴더를 가리킴)
set(EIGEN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen3)
include_directories(${EIGEN_PATH})

# -----------------------------------------------------------
# 2. MuJoCo 찾기 (NEW!)
# MuJoCo가 설치된 경로를 CMake에게 알려줍니다.
# set(mujoco_DIR $ENV{HOME}/.mujoco/mujoco-3.4.0)

set(MUJOCO_ROOT "/home/kim/.mujoco/mujoco-3.4.0")

# 헤더 파일(.h) 위치: 보통 mujoco-3.4.0/include 에 있습니다.
#include_directories(${MUJOCO_ROOT}/include)

# 라이브러리 파일(.so) 위치: 방금 확인해주신 그 폴더입니다!
#link_directories(${MUJOCO_ROOT}/lib)

# 3. GLFW (화면 도구) 찾기 (NEW!)
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
# -----------------------------------------------------------

# 4. 실행 파일 만들기 (sim_test)
add_executable(sim_test src/sim_test.cpp)

#5. [FIX] target_include_directories를 사용하여 더 강력하게 연결합니다.
target_include_directories(sim_test PRIVATE 
    ${MUJOCO_ROOT}/include
    ${MUJOCO_ROOT}/include/mujoco
) 

# 6. 라이브러리 링크
# [FIX] 라이브러리 폴더 지정
target_link_directories(sim_test PRIVATE ${MUJOCO_ROOT}/lib)

# 5. 재료 섞기 (Link)
# Eigen은 헤더만 있어서 링크 불필요, MuJoCo와 GLFW는 링크 필수!
target_link_libraries(sim_test 
    PRIVATE 
    mujoco
    ${GLFW_LIBRARIES}
)

# 6. 설치 (서빙)
install(TARGETS sim_test DESTINATION lib/${PROJECT_NAME})

# ★중요★ 모델 파일(xml)도 install 폴더로 같이 복사해야 실행할 때 읽을 수 있습니다.
install(DIRECTORY model DESTINATION share/${PROJECT_NAME})