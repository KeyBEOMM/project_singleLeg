# 1. CMake 최소 버전 설정 (더 낮은 버전 사용 시 문법 호환성 문제 발생 가능)
cmake_minimum_required(VERSION 3.10)

# 2. 패키지 이름 설정
project(test) # 패키지 이름은 pacekage.xml과 동일해야 한다.

# 3. 빌드 도구 설정 
# 왜 필요한가: ROS 2 빌드 시스템(ament)의 매크로들을 불러옵니다.
# 핵심: colcon을 통해 패키지 간의 경로를 관리하고 source install/setup.bash를 가능하게 하는 핵심 도구입니다.
find_package(ament_cmake REQUIRED)

# 4. core_lib 패키지 찾기 (앞서 빌드한 core_lib 패키지가 어디에 설치되어 있는지 찾음)
# core_lib가 ament_export_libraries를 통해 자신을 수출했기 때문에, 여기서 "이름"만으로 찾을 수 있는 것입니다.
find_package(core_lib REQUIRED)

# 4-1 소스파일 경로 설정 (필요시)
# set 

# 5. 실행 파일 생성
add_executable(KF_test src/KF_test.cpp)
add_executable(verify_kinematics src/kinematics_verify.cpp)

# 5-1 헤더 파일 경로 포함 (필요시) 패키지 로컬 헤더가 없으므로 생략
# target_include_directories
    
# 6. 라이브러리 물리적 연결 (Linking)
target_link_libraries(KF_test 
    core_lib::core_lib   
)
target_link_libraries(verify_kinematics 
    core_lib::core_lib   
)

# 7 install ROS 2는 빌드 결과물을 반드시 install 영역으로 옮겨야 실행 도구가 찾을 수 있다. 개발용이라고 설치를 빼먹으면 안 된다.
install(TARGETS KF_test verify_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

# 8. ament 패키징 마무리
ament_package()