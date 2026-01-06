cmake_minimum_required(VERSION 3.10)
project(simulation)

find_package(ament_cmake REQUIRED)

find_package(core_lib REQUIRED)

set(MUJOCO_PATH "$ENV{HOME}/.mujoco/mujoco-3.4.0")

find_package(mujoco REQUIRED CONFIG PATHS ${MUJOCO_PATH})

find_package(glfw3 REQUIRED)

add_executable(sim_test sim_test.cpp)

target_link_libraries(sim_test 
    core_lib::core_lib 
    mujoco::mujoco 
    glfw3::glfw3
)
# 7. ament 패키징 마무리
ament_package()

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)



# set(MUJOCO_ROOT "/home/kim/.mujoco/mujoco-3.4.0")




# # 3. GLFW (화면 도구) 찾기 (NEW!)
# find_package(PkgConfig REQUIRED)
# pkg_search_module(GLFW REQUIRED glfw3)
# # -----------------------------------------------------------

# # 4. 실행 파일 만들기 (sim_test)
# add_executable(sim_test src/sim_test.cpp)

# #5. [FIX] target_include_directories를 사용하여 더 강력하게 연결합니다.
# target_include_directories(sim_test PRIVATE 
#     ${MUJOCO_ROOT}/include
#     ${MUJOCO_ROOT}/include/mujoco
# ) 

# # 6. 라이브러리 링크
# # [FIX] 라이브러리 폴더 지정
# target_link_directories(sim_test PRIVATE ${MUJOCO_ROOT}/lib)

# # 5. 재료 섞기 (Link)
# # Eigen은 헤더만 있어서 링크 불필요, MuJoCo와 GLFW는 링크 필수!
# target_link_libraries(sim_test 
#     PRIVATE 
#     mujoco
#     ${GLFW_LIBRARIES}
# )

# # 6. 설치 (서빙)
# install(TARGETS sim_test DESTINATION lib/${PROJECT_NAME})

# # ★중요★ 모델 파일(xml)도 install 폴더로 같이 복사해야 실행할 때 읽을 수 있습니다.
# install(DIRECTORY model DESTINATION share/${PROJECT_NAME})