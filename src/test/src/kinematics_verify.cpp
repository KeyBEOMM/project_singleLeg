#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <iomanip>
#include "core_lib/kinematics.h"

// 색상 출력을 위한 매크로
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"

void printVec3(const std::string& name, const Eigen::Vector3d& v) {
    std::cout << name << ": [" 
              << std::fixed << std::setprecision(4) << v(0) << ", " 
              << v(1) << ", " << v(2) << "]";
}

int main() {
    Leg leg;
    
    // 랜덤 엔진 설정
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // 관절 제한 범위 (HAA, HFE, KFE) - kinematics.h의 값 참고
    // HAA: -0.78 ~ 2.09
    // HFE: -1.57 ~ 2.79
    // KFE: -2.70 ~ 2.70
    std::uniform_real_distribution<double> dist_q1(-0.5, 0.5);   // HAA (안전 범위)
    std::uniform_real_distribution<double> dist_q2(-0.5, 1.5);   // HFE
    std::uniform_real_distribution<double> dist_q3(-2.0, -0.5);  // KFE (무릎 굽힘)

    int num_tests = 50;
    int success_count = 0;

    std::cout << "=============================================================\n";
    std::cout << "       Kinematics Verification Test (FK -> Analytical IK)    \n";
    std::cout << "=============================================================\n";

    for (int i = 0; i < num_tests; ++i) {
        // 1. 랜덤 관절 각도 생성 (Target Angles)
        Eigen::Vector3d q_target;
        q_target << dist_q1(gen), dist_q2(gen), dist_q3(gen);

        // 2. FK 실행 (Joint -> Position)
        Eigen::Matrix4d T_target = leg.bodyframe_PoE(q_target);
        Eigen::Vector3d pos_target = T_target.block<3,1>(0,3);

        // 3. IK 실행 (Position -> Joint)
        // Analytical IK는 주로 무릎이 한쪽으로 굽혀지는 해를 반환하므로 
        // q_target 설정 시 KFE 부호에 유의해야 검증이 쉬움.
        Eigen::Vector3d q_sol = leg.bodyframe_ANL(T_target);

        // 4. 검증
        // 4-1. 관절 각도 오차 확인
        double q_error = (q_target - q_sol).norm();

        // 4-2. 위치 오차 확인 (IK로 구한 각도로 다시 FK를 했을 때 위치)
        Eigen::Matrix4d T_sol = leg.bodyframe_PoE(q_sol);
        Eigen::Vector3d pos_sol = T_sol.block<3,1>(0,3);
        double pos_error = (pos_target - pos_sol).norm();

        bool is_success = (pos_error < 1e-4);

        if (is_success) {
            success_count++;
            std::cout << "[Test " << std::setw(2) << i << "] " << GREEN << "PASS" << RESET;
            std::cout << "[result] " << leg.getJoint_Angle_Vector().transpose() << std::endl;
        } else {
            std::cout << "[Test " << std::setw(2) << i << "] " << RED << "FAIL" << RESET;
        }

        std::cout << " | Pos Err: " << std::setprecision(6) << pos_error;
        std::cout << " | Ang Err: " << std::setprecision(6) << q_error;
        
        if (!is_success || q_error > 0.1) {
            std::cout << "\n    -> Target Q: " << q_target.transpose();
            std::cout << "\n    -> Solved Q: " << q_sol.transpose();
        }
        std::cout << std::endl;
    }

    std::cout << "=============================================================\n";
    std::cout << "Result: " << success_count << " / " << num_tests << " passed.\n";
    std::cout << "=============================================================\n";

    return 0;
}
