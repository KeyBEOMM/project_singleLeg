## ament cmake란?
ament_cmake는 ROS 2에서 CMake 기반 패키지(주로 C/C++ 프로젝트)를 빌드하기 위한 핵심 빌드 시스템입니다. 표준 CMake 기능을 확장하여 ROS 2 환경에 특화된 편의 기능과 매크로를 제공합니다. 

## CMAKE build
   단계             주요 명령어                      핵심 역할
1. 찾기 (Search),   find_package(),               "시스템에 설치된 외부 라이브러리(Eigen, MuJoCo 등)의 경로와 정보를 가져옴"
2. 생성 (Produce),  add_library(),                 내가 쓴 .cpp 파일들을 묶어서 하나의 바이너리(.so 또는 .a)로 만듦
3. 설정 (Setting),  target_include_directories(),  내 라이브러리가 사용할 헤더(.h) 폴더 위치를 컴파일러에게 알려줌
4. 연결 (Link),     target_link_libraries(),       내 라이브러리가 외부 라이브러리의 기능을 빌려 쓸 수 있게 물리적으로 연결함
5. 수출 (Export),   install(EXPORT...),            다른 패키지가 find_package()를 통해 나를 찾을 수 있도록 설정 파일을 만듦

💡 최상위 CMake의 역할: 여러 패키지가 엮여 있을 때 add_subdirectory()를 통해 빌드 트리에 포함시키고, 의존성에 따라 어떤 폴더를 먼저 요리할지 순서를 정하는 '총괄 주방장' 역할을 합니다.

## COLCON build 
Colcon은 CMake보다 한 단계 더 높은 곳에서 여러 개의 독립된 패키지들을 관리하는 도구입니다. 가장 큰 특징은 **"최상위 CMakeLists.txt가 필요 없다"**는 점입니다.

① Colcon의 작업 공간(Workspace) 구조
Colcon은 개별 패키지들이 모인 src 폴더를 통째로 관리합니다.

my_workspace/             # 최상위 (여기서 colcon build 실행)
├── src/                  # 소스 폴더 (유일하게 사용자가 건드리는 곳)
│   ├── core_lib/         # 패키지 A (package.xml + CMakeLists.txt)
│   └── test/             # 패키지 B (package.xml + CMakeLists.txt)
├── build/                # 빌드 중 발생하는 중간 파일 (자동 생성)
├── install/              # 빌드 완료된 결과물 (자동 생성, 여기가 핵심!)
└── log/                  # 빌드 로그

② xml이 필요한 이유
1.  colcon build 시 colcon은 모든 xml 파일을 읽어 패키지의 의존성 정보를 파악하고, 필요한 빌드 순서를 결정.
2.  xml에 의존성 정보를 작성하여 cmakelist에서 find_package 명령어가 정상 작동하여 라이브러리 경로와 헤더 경로를 찾을 수 있게된다..-->

③ 실행 환경의 분리: install과 source
Colcon 빌드가 끝나면 install 폴더에 모든 패키지의 라이브러리와 실행 파일이 모입니다. 하지만 컴퓨터는 아직 이 파일들이 어디 있는지 모릅니다.

source install/setup.bash: 이 명령어를 치는 순간, install 폴더 내의 경로들이 시스템 환경 변수(PATH, LD_LIBRARY_PATH)에 등록됩니다.

이후에는 어느 폴더에 있든 상관없이 내가 만든 패키지를 찾아서 실행하거나 다른 프로젝트에서 불러올 수 있게 됩니다.

### CMakeLists.txt
CMakeLists파일은 동일하게 build 시 빌드 레시피의 역할을 하여 colcon build라는 자동 빌드 툴로 install 폴더에 담기게 된다.
실행 시 setup.bash를 통해 환경 변수로 가져와 실행 할 수 있도록 한다.

- 작성 방법
순서,   섹션            명칭,       설명
1단계,  Preamble,      "CMake 버전 및 프로젝트 이름 선언 (cmake_minimum_required, project)"
2단계,  Dependencies,   필요한 외부/내부 패키지 찾기 (find_package)
3단계,  Build Targets, "실행 파일이나 라이브러리 정의 (add_executable, add_library)"
4단계,  Linking,타겟과   라이브러리 연결 (target_link_libraries)
5단계,  Packaging,     "설치 규칙 정의 및 마무리 (install, ament_package)"