cmake_minimum_required(VERSION 3.10)
project(core_lib)

# 1. 빌드 환경 설정
find_package(ament_cmake REQUIRED)

find_package(Eigen3 REQUIRED)

# 3. 라이브러리 생성
add_library(${PROJECT_NAME} SHARED
  src/kinematics.cpp # cmakelists.txt 기준 상대 경로
)

# 4. 헤더 파일 경로 인터페이스로 지정 (중요!)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> # kinematics.h 위치 cmake_current_source_dir는 이 CMakeLists.txt가 있는 src/core_lib 폴더를 가르킴
  $<INSTALL_INTERFACE:include>
)

# 5. 의존성 연결 (외부 라이브러리를 사용하기에 의존성 전파 및 링킹을 위해 필요)
# PUBLIC: 내 헤더(kinematics.h)에 Eigen 코드가 있으므로, 나를 쓰는 놈도 알게 함.
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

# 5. 다른 패키지를 위한 내보내기(Export)
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3) # 나를 쓰는 놈도 Eigen을 알아야 함 (의존성 전파)
ament_export_include_directories(include)

# (1) 헤더 파일 설치
install(
  DIRECTORY include/
  DESTINATION include
)

# (3) 라이브러리 파일(.so) 및 CMake 설정 파일 설치
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()