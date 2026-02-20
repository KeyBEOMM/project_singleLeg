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

    std::cout << "=============================================================";
    std::cout << "       Robustness / Error Handling Tests                     ";
    std::cout << "       (Expecting IK to fail and return initial/safe state)  ";
    std::cout << "=============================================================";

    int error_pass_count = 0;
    int total_error_tests = 30; // 10 * 3

    // 1. Out of Reach Tests
    std::cout << "[Type 1: Out of Reach] (Target Dist > 0.40m)";
    for(int i=0; i<10; ++i) {
        Eigen::Vector3d origin_value = leg.getJoint_Angle_Vector(); // 초기값 저장

        Eigen::Vector3d dir = Eigen::Vector3d::Random().normalized();
        double dist = 0.45 + (std::abs(dir[0]) * 0.5); // Min 0.45m
        Eigen::Vector3d target = dir * dist;
        
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,1>(0,3) = target;

        // IK 시도
        Eigen::Vector3d q_res = leg.bodyframe_ANL(T);
        bool is_failed = (leg.getJoint_Angle_Vector() == origin_value); // 또는 q_res가 target과 확연히 다른지 확인
        

        // 에러가 커야 성공 (도달 못함)
        if (is_failed) { 
            std::cout << "[Case " << i << "] " << GREEN << "PASS" << RESET << " (Dist: " << std::fixed << std::setprecision(3) << dist << ", Failed: " << is_failed << ")";
            std::cout << " | Target: [" << q_res.transpose() << "]" << " | Ori: [" << origin_value.transpose() << "]" << std::endl;
            error_pass_count++;
        } else {
            std::cout << "[Case " << i << "] " << RED << "FAIL" << RESET << " (Reached? Failed: " << is_failed << ")";
            std::cout << " | Target: [" << q_res.transpose() << "]" << " | Ori: [" << origin_value.transpose() << "]" << std::endl;
        }
    }

    // 2. Deadzone Tests
    std::cout << "[Type 2: Inside Deadzone] (YZ Dist < 0.04m)";

    for(int i=0; i<10; ++i) {
        Eigen::Vector3d origin_value = leg.getJoint_Angle_Vector(); // 초기값 저장

        // YZ 평면 상에서 HAA Offset(0.04) 보다 작은 위치 생성
        std::uniform_real_distribution<double> dist_angle(-3.14, 3.14);
        std::uniform_real_distribution<double> dist_rad(0.0, 0.035); // < 0.04
        
        double theta = dist_angle(gen);
        double r_yz = dist_rad(gen); 
        
        double y = r_yz * cos(theta);
        double z = r_yz * sin(theta);
        double x = std::uniform_real_distribution<double>(0.1, 0.3)(gen);

        Eigen::Vector3d target(x, y, z);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,1>(0,3) = target;

        Eigen::Vector3d q_res = leg.bodyframe_ANL(T);
        
        bool is_failed = (leg.getJoint_Angle_Vector() == origin_value); // 또는 q_res가 target과 확연히 다른지 확인
        if (is_failed) { 
            std::cout << "[Case " << i << "] " << GREEN << "PASS" << RESET << " (YZ-Dist: " << r_yz << ", Failed: " << is_failed << ")";
            std::cout << " | Target: [" << q_res.transpose() << "]" << " | Ori: [" << origin_value.transpose() << "]" << std::endl;
            error_pass_count++;
        } else {
            std::cout << "[Case " << i << "] " << RED << "FAIL" << RESET << " (Reached? Failed: " << is_failed << ")";
            std::cout << " | Target: [" << q_res.transpose() << "]" << " | Ori: [" << origin_value.transpose() << "]" << std::endl;
        }
    }

    // 3. Joint Limits (Random Joint Limit Violation)
    std::cout << "[Type 3: Joint Limits] (Target from invalid angles)";
    
    // 각 관절의 Valid 범위 (kinematics.h 참조)
    // q0: -0.7854 ~ 2.0944
    // q1: -1.5708 ~ 2.7925
    // q2: -2.7050 ~ 2.7050
    double limits[3][2] = {
        {-0.7854, 2.0944},
        {-1.5708, 2.7925},
        {-2.7050, 2.7050}
    };

    for(int i=0; i<10; ++i) {
        Eigen::Vector3d origin_value = leg.getJoint_Angle_Vector(); // 초기값 저장

        // 1. 랜덤하게 하나의 관절을 선택해 범위를 벗어나게 설정
        Eigen::Vector3d q_invalid;
        int viol_idx = std::uniform_int_distribution<int>(0, 2)(gen); // 0, 1, or 2

        for(int j=0; j<3; ++j) {
            if(j == viol_idx) {
                // 범위 밖 값 생성 (Min보다 작거나 Max보다 크게)
                if (std::uniform_int_distribution<int>(0, 1)(gen)) {
                    q_invalid[j] = limits[j][1] + std::uniform_real_distribution<double>(0.1, 0.5)(gen); // Max + alpha
                } else {
                    q_invalid[j] = limits[j][0] - std::uniform_real_distribution<double>(0.1, 0.5)(gen); // Min - alpha
                }
            } else {
                // 나머지는 안전 범위 내 생성
                double mid = (limits[j][0] + limits[j][1]) / 2.0;
                q_invalid[j] = mid; 
            }
        }

        // 2. 해당 각도로 FK 계산 -> Target Pose
        Eigen::Matrix4d T_target = leg.bodyframe_PoE(q_invalid);
        Eigen::Vector3d target_pos = T_target.block<3,1>(0,3);

        // 3. IK 실행
        Eigen::Vector3d q_res = leg.bodyframe_ANL(T_target);
        
        bool is_failed = (leg.getJoint_Angle_Vector() == origin_value); // 또는 q_res가 target과 확연히 다른지 확인
        if (is_failed) {
            std::cout << "[Case " << i << "] " << GREEN << "PASS" << RESET << " (Violated Joint: q" << viol_idx << ", Failed: " << is_failed << ")";
            std::cout << " | Target: [" << q_res.transpose() << "]" << " | Ori: [" << origin_value.transpose() << "]" << std::endl;
            error_pass_count++;
        } else {            
            std::cout << "[Case " << i << "] " << RED << "FAIL" << RESET << " (Reached? Failed: " << is_failed << ")";
            std::cout << " | Target: [" << q_res.transpose() << "]" << " | Ori: [" << origin_value.transpose() << "]" << std::endl;
        }
    }
        
    return 0;
}
