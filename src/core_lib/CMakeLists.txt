# 1. 시스템 find_package(Eigen3)를 사용하지 않습니다.

# 2. 내부에 저장된 Eigen 경로를 직접 지정합니다.
# ${CMAKE_CURRENT_SOURCE_DIR}는 현재 core_lib 폴더 위치를 가리킵니다.
set(LOCAL_EIGEN_PATH ${CMAKE_CURRENT_SOURCE_DIR}/third_party/eigen3)

# 3. 라이브러리 빌드 시 이 경로를 포함시킵니다.
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${LOCAL_EIGEN_PATH}>
  $<INSTALL_INTERFACE:include>
)

# 4. 다른 패키지(simulation 등)에 이 경로를 수출(Export)합니다.
ament_export_include_directories(include ${LOCAL_EIGEN_PATH})