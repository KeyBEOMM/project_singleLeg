#include <iostream>
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h> 

// MuJoCo 데이터 구조체
mjModel* m = NULL;   // 모델 (변하지 않는 것: 로봇 길이, 무게 등)
mjData* d = NULL;    // 데이터 (변하는 것: 위치, 속도 등)

int main(int argc, char** argv) {
    // 1. XML 모델 로드 (경로 주의!)
    // 실행 시 install 폴더 기준으로 경로를 찾습니다.
    const char* filename = "install/my_robot_core/share/my_robot_core/model/test.xml";
    if (argc > 1) filename = argv[1];

    char error[1000];
    m = mj_loadXML(filename, 0, error, 1000);
    if (!m) {
        std::cerr << "Load Error: " << error << std::endl;
        return 1;
    }
    d = mj_makeData(m);

    // 2. 그래픽 창(Window) 초기화
    if (!glfwInit()) return 1;
    GLFWwindow* window = glfwCreateWindow(1200, 900, "My MuJoCo Sim", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 3. 시각화 설정 (카메라, 조명 등)
    mjvCamera cam; mjv_defaultCamera(&cam);
    mjvOption opt; mjv_defaultOption(&opt);
    mjvScene scn;  mjv_defaultScene(&scn);
    mjv_makeScene(m, &scn, 2000); // <--- ★이 줄을 꼭 추가해주세요!★ (최대 2000개 물체 그리기 준비)
    mjrContext con; mjr_defaultContext(&con);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 4. 시뮬레이션 루프 (무한 반복)
    while (!glfwWindowShouldClose(window)) {
        // 물리 연산 (Step)
        mj_step(m, d);

        // 화면 그리기 (Render)
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // 종료 정리
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    glfwTerminate();
    return 0;
}