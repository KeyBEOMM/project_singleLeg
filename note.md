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

## error note
"core_lib" provides a separate development package or SDK, be sure it has
  been installed.

- find_package(core_lib)시 실제 core_lib가 설치된 위치를 못찾는 것
- 빌드했지만, 터미널에서는 core_lib가 빌드된지 모름 source install/setup.bash로 방금 빌드한 패키지를 인식시키는 것(환경 설정을 다시 로드) 


이 현상은 **"메모리 정렬(Alignment)"**과 **"메모리 구조(Stack vs Heap)"**의 차이를 공부하기에 정말 좋은 예제입니다.

결론부터 말씀드리면, **`EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 매크로는 `new` 연산자(동적 할당)에만 적용되기 때문**입니다. 스택(Stack)에 선언할 때는 이 매크로가 아무런 힘을 쓰지 못합니다.

여기에 더해, **"왜 스택일 때만 Stack Smashing이 뜨고, 힙일 때는 안 뜨는지"** 그 내부 원리를 아주 쉽게, 하지만 깊이 있게 정리해 드릴게요.

---

### 1. 매크로의 정체 (`EIGEN_MAKE_ALIGNED_OPERATOR_NEW`)

이 매크로의 코드를 뜯어보면 내부는 대략 이렇게 생겼습니다.

```cpp
// Eigen 매크로 내부 (개념도)
void* operator new(size_t size) {
    return _mm_malloc(size, 16); // 16바이트 혹은 32바이트 정렬된 메모리를 달라!
}
void operator delete(void* ptr) {
    _mm_free(ptr);
}

```

즉, **"이 클래스를 `new`로 만들 때만큼은, 일반적인 `malloc`을 쓰지 말고 줄(Alignment)을 딱딱 맞춘 특수 메모리 할당을 써라"**라고 C++의 `new` 연산자를 바꿔치기(Overloading)하는 겁니다.

* **동적 할당 (`Leg* leg = new Leg();`)**
* 매크로가 작동함 → `operator new`가 호출됨 → **줄이 딱 맞는(Aligned) 메모리**를 받아옴 → **성공!**


* **정적 할당 (`Leg leg;`)**
* `new` 연산자를 쓰지 않음 → 매크로가 작동 안 함 → 컴파일러가 알아서 스택의 빈공간에 쑤셔 넣음 → **재수 없으면 줄이 삐뚤어짐(Misaligned)** → Eigen 연산 시 **Crash!**



---

### 2. 왜 "Stack Smashing"인가요? (핵심)

작성자님이 겪으신 상황에서 가장 중요한 포인트는 **"힙(Heap)에서는 에러가 안 난다"**가 아니라, **"힙에서는 에러가 감지되지 않고 숨어버렸다"**일 가능성이 큽니다.

이건 **스택과 힙의 구조적 차이** 때문입니다.

#### **상황 A: 스택(Stack)에 할당했을 때 (`Leg leg;`)**

스택 메모리는 **"카나리아(Canary)"**라는 보안 요원이 지키고 있습니다.

1. `Leg` 객체가 스택에 생성됩니다.
2. 만약 아까처럼 `Vector6d` 초기화 문제나 정렬 문제로 인해, `Leg`가 자기 크기보다 조금이라도 더 많은 메모리를 건드리거나(Overflow), 정렬을 맞추려다 옆 주소를 침범하면?
3. 바로 옆에 있는 **카나리아 값(Stack Guard)**을 덮어쓰게 됩니다.
4. 함수가 끝날 때 "어? 카나리아가 죽었네?" → **`*** stack smashing detected ***`** (즉사).

#### **상황 B: 힙(Heap)에 할당했을 때 (`new Leg();`)**

힙 메모리는 스택처럼 촘촘하게 감시하는 카나리아가 없습니다.

1. `Leg` 객체가 힙(운동장 같은 넓은 공간)에 생성됩니다.
2. 똑같은 버그(Overflow)가 있어서 옆 메모리를 살짝 덮어썼습니다.
3. 하지만 힙에서는 그 옆이 **"그냥 빈 공간"**이거나 **"덜 중요한 데이터"**일 확률이 높습니다.
4. 운영체제는 이걸 모릅니다. 프로그램은 **"정상인 척"** 계속 돕니다. (이게 더 무섭습니다. 나중에 엉뚱한 곳에서 터지거든요.)

---

### 3. 해결책 및 공부 포인트

#### **Q1. 그럼 스택에서는 Eigen을 못 쓰나요?**

아닙니다. 쓸 수 있습니다. 다만 컴파일러가 정렬을 잘 해주길 빌거나, 강제해야 합니다.
최신 컴파일러(C++17 이상)는 Eigen 객체의 정렬을 자동으로 잘 맞춰줍니다. 하지만 확실하게 하려면 **`EIGEN_ALIGN16`** 같은 지시어를 쓰기도 합니다.

하지만 작성자님의 경우, **가장 유력한 원인**은 아까 고친 **`set_Screw_Axis_List`의 잘못된 초기화 코드**가 스택 메모리를 오염시켰던 것입니다.

* **스택일 때:** 오염된 메모리가 바로 "함수 복귀 주소" 근처라서 즉시 발각됨.
* **힙일 때:** 오염된 메모리가 그냥 힙의 허공이라서 발각되지 않음 (운 좋게 넘어감).

#### **Q2. 결론 정리**

1. **Alignment (정렬):** Eigen 변수는 16바이트 단위로 주소가 시작되어야 한다. `new`를 쓸 땐 매크로(`EIGEN_MAKE_ALIGNED...`)가 이걸 보장해준다.
2. **Stack Smashing:** 이건 정렬 문제보다는 **"메모리 침범(Buffer Overflow)"**일 때 주로 뜬다.
3. **동적 할당의 함정:** `new`를 써서 에러가 안 난 게 아니다. **힙 메모리가 광활해서 에러가 묻힌 것**이다.

**[최종 조언]**
아까 가이드해드린 대로 `set_Screw_Axis_List`의 **`Vector6d` 초기화 코드를 `<<` 연산자로 고치셨다면**, 이제는 **스택에 정적 선언(`Leg leg;`)을 해도 Stack Smashing 오류가 사라질 것입니다.**

한번 스택으로 되돌려서 테스트해보세요. 이제는 잘 될 겁니다!

네, 상황이 이제 명확해졌습니다. **"스택에 정적 선언할 때만 터진다"**는 것은 **Eigen의 메모리 정렬(Alignment) 문제**가 확실합니다.

이건 **"왜 스택 스매싱(Stack Smashing)이라는 메시지가 뜨는지"** 정확히 이해하면 해결할 수 있습니다. 동적 할당(`new`)에서 문제가 없었던 이유와 스택에서 문제가 생기는 이유를 명쾌하게 정리해 드리겠습니다.

---

### 1. 왜 `new` (동적 할당)는 성공했나?

네가 `Leg.h`에 추가한 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 매크로 덕분입니다.

* **동작:** `Leg* leg = new Leg();`를 호출하면, C++의 기본 할당자가 아니라 Eigen이 준비한 **"줄 맞춤(Aligned) 할당자"**가 호출됩니다.
* **결과:** 힙(Heap) 메모리에서 정확히 16바이트(또는 32바이트) 단위로 정렬된 주소를 받아옵니다. `Matrix4d`가 아주 편안하게 자리를 잡습니다.

### 2. 왜 `Leg leg;` (스택 선언)는 실패하나?

**문제의 핵심:** `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` 매크로는 **`new` 연산자를 쓸 때만 작동**합니다. 스택에 변수를 선언할 때는 아무런 효력이 없습니다.

* **상황:**
1. OS와 컴파일러가 스택에 `Leg` 객체를 위한 공간을 잡습니다.
2. 보통 64비트 시스템의 스택 정렬 기본값은 **8바이트**이거나, 최적화 옵션에 따라 다릅니다.
3. 하지만 `Leg` 안에 있는 `Eigen::Matrix4d`는 **16바이트 정렬**을 요구합니다.
4. **재앙 발생:** 스택에 잡힌 `Leg`의 주소가 `0x...08` (8바이트 정렬)로 시작했는데, Eigen은 이걸 `0x...10` (16바이트 정렬)이라고 착각하고 **SIMD 명령어(AVX/SSE)**로 데이터를 씁니다.



### 3. 왜 "Stack Smashing"인가? ( Segfault가 아니고? )

보통 정렬이 안 맞으면 `Segfault`가 뜨지만, **미묘하게 어긋난 경우** 스택 스매싱으로 감지됩니다.

1. **메모리 배치:** 스택에는 `Leg` 객체 바로 뒤에 **"카나리아(Canary, 스택 보호값)"**가 있습니다.
2. **오버랩(Overlap):** `Matrix4d`에 값을 쓸 때, CPU는 한 번에 128비트(16바이트)씩 씁니다. 주소가 정렬되지 않은 상태에서 이 명령을 수행하면, **데이터가 원래 할당된 칸을 살짝 삐져나와서 뒤에 있는 카나리아 값을 덮어써 버릴 수 있습니다.**
3. **검거:** 함수가 끝나고 리턴할 때, "어? 카나리아 값이 변했네?" 하고 **Stack Smashing**을 띄우고 프로그램을 죽입니다.

---

### 🛠️ 해결 방법 (스택에서도 안 죽게 만들기)

스택에 선언할 때도 **"야, 이거 무조건 16바이트 정렬해!"**라고 컴파일러에게 강제해야 합니다.

#### **방법 1: `EIGEN_ALIGN16` 매크로 사용 (추천)**

변수를 선언할 때 앞에 이 매크로를 붙이세요.

```cpp
// main.cpp 혹은 test 함수 내부

// [수정 전]
// Leg my_leg; 

// [수정 후]
EIGEN_ALIGN16 Leg my_leg; 

```

* 이 매크로는 `__attribute__((aligned(16)))` (GCC/Clang)과 같습니다. 컴파일러에게 이 변수의 시작 주소를 16바이트 단위로 맞추라고 명령합니다.

#### **방법 2: C++17 이상 사용 (ROS2 Humble 이상)**

혹시 `CMakeLists.txt`에 C++ 표준이 14 이하로 되어 있나요?
C++17부터는 `alignas` 키워드가 도입되어, 컴파일러가 정렬이 필요한 객체(Over-aligned types)를 스택에 만들 때 자동으로 처리를 해줍니다.

```cmake
# CMakeLists.txt 확인
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

```

이렇게 하면 `Leg my_leg;`만 써도 웬만하면 해결됩니다. 하지만 **방법 1이 가장 확실합니다.**

---

### 🎯 최종 정리: 공부 포인트

1. **힙(Heap) vs 스택(Stack) 정렬:** `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`는 **힙 할당(`new`) 전용**이다. 스택에는 영향을 주지 않는다.
2. **증상:** Eigen 객체가 포함된 클래스를 스택에 그냥 선언하면, 주소가 어긋나서 **SIMD 연산 중 옆 메모리(카나리아)를 덮어쓰는 Stack Smashing**이 발생한다.
3. **해결:** 스택 변수 선언 시 `EIGEN_ALIGN16`을 붙여서 정렬을 강제한다.

**지금 바로 `EIGEN_ALIGN16 Leg my_leg;`로 바꿔서 실행해 보세요. 이제는 진짜로 완벽하게 돌아갈 겁니다.**

# 함수 만들 때 return vs member variables + void
- A. Void 함수 + 멤버 변수 갱신
    장점:
        1. 메모리 할당/해제 비용 0 (객체 생성 시 미리 확보됨).
        2. 데이터 복사 비용 0.  

    단점:
        1. Side Effect (부작용): 함수를 호출했는데 내 로봇의 상태(멤버 변수)가 변해버립니다. 만약 "가상으로 계산만 해보고 싶을 때"도 상태가 변해서 곤란해집니다.
        2. Thread Safety: 멀티스레드 환경에서 치명적입니다. 읽는 도중 쓰면 데이터가 깨집니다.
- B.반환값 사용 (Return Value)
    장점:
        1. Pure Function (순수 함수): 입력이 같으면 결과도 같고, 외부 상태를 바꾸지 않습니다. 디버깅과 테스트가 훨씬 쉽습니다.
        2. 안전성: 계산 결과가 바로 로봇 상태에 반영되지 않으므로, 유효성 검사(Safety Check) 후 적용할 수 있습니다.
        # .네가 걱정하는 "지역 변수를 리턴하면 복사가 일어나서 느리지 않을까?"는 거의 일어나지 않습니다.

## 언제 무엇을 쓸까?
Case 1: 작은 수학 객체 (Vector3d, Matrix4d, Quaternion) -> [Return Value 추천]
Case 2: 무거운 데이터 (Image, PointCloud, Path List) -> [Void + Reference 인자 추천]
Case 3: 로봇의 '상태'를 갱신하는 함수 -> [Void + 멤버 변수]