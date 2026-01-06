cmake_minimum_required(VERSION 3.10)
project(core_lib)

# 1. 빌드 환경 설정
find_package(ament_cmake REQUIRED)

# 2. 로컬 Eigen3 경로 설정 (third_party 폴더 내의 경로)
# 현재 CMakeLists.txt 위치에서 상대 경로로 잡습니다.
set(EIGEN_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen3)

# 3. 라이브러리 생성
add_library(leg_fk_lib leg_FK.cpp)

# 4. 헤더 파일 경로 포함 (중요!)
target_include_directories(leg_fk_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/Forward_Kinematics_Algorithm> # leg_FK.h 위치 cmake_current_source_dir는 이 CMakeLists.txt가 있는 src/core_lib 폴더를 가르킴
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>
)

# 5. 다른 패키지를 위한 내보내기(Export)
# 나를 가져다 쓰는 simulation 패키지가 Eigen 경로를 자동으로 알게 해줍니다.
# ament_export_include_directories(Forward_Kinematics ${EIGEN_INCLUDE_DIR})
# ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_leg_fk_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(ament_cmake)


# 6. 설치 규칙
install(TARGETS leg_fk_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()