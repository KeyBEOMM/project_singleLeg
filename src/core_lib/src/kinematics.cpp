#include "core_lib/kinematics.h"

/*
고민해볼 점
0. 동적 메모리 할당 최소화 -> Eigen::Matrix4d Transformation_Mat = Eigen::Matrix4d::Zero(); 등 불필요한 할당 제거 + std::vector x
    (가장 시급한 문제입니다. 로봇 제어 루프(Real-time loop)에서는 **"절대 힙(Heap) 메모리 할당이 일어나서는 안 된다"**는 철칙이 있습니다.)
1. 링크의 수가 변하면 사용 불가 -> 링크 수에 상관없이 동작하도록 변경 필요
2. home matrix와 screw axis list가 하드코딩 되어있음 -> config 파일로부터 불러오도록 변경 필요
3. 로드리게스 formula에서 cos, sin 계산이 중복 -> 미리 계산해두고 재사용하도록 변경 필요
4. error handling 없음 -> 추후 추가 필요
5. 추가로 계산 구현 시에 연산 순서가 수식적으로 상관없을 때 어떤 순서로 계산하는 것이 더 효율적인지 고민해볼 것(항상 행렬을 작게 만드는 것을 우선)

*/

Leg::Leg() {
    // Initialize the home matrix
    set_Home_Matrix();

    robot_state.T = M_home;

    // Initialize the screw axis list
    set_Screw_Axis_List();
    set_BodyScrew_Axis_List();
}

Leg::~Leg() {
}

// Set the home configuration matrix M
void Leg::set_Home_Matrix() {

    M_home.setIdentity();
    M_home(1, 3) = HAA_Offset;
    M_home(2, 3) = -(Upper_Link + Lower_Link);
}

// Set the screw axis list for the leg joints 현재 하드코딩 되어있는데 이후 config 파일로부터 불러오도록 변경 필요
void Leg::set_Screw_Axis_List() {
    // Define the screw axes for the leg joints
    // Q. is it better to set parameters as variables and assign these values for other functions's result?
    Eigen::Vector3d q1, q2, q3; // points on the axes
    Eigen::Vector3d w1, w2, w3; // direction of the
    Eigen::Vector3d v1, v2, v3; // linear velocity components

    q1 = Eigen::Vector3d(0, 0, 0);
    q2 = Eigen::Vector3d(0, HAA_Offset, 0);
    q3 = Eigen::Vector3d(0, HAA_Offset, -Upper_Link);
    w1 = Eigen::Vector3d(1, 0, 0);
    w2 = Eigen::Vector3d(0, 1, 0);
    w3 = Eigen::Vector3d(0, 1, 0);
    v1 = -w1.cross(q1);
    v2 = -w2.cross(q2);
    v3 = -w3.cross(q3);
    // S_axis.col(0) = Vector6d(w1(0), w1(1), w1(2), v1(0), v1(1), v1(2));
    // S_axis.col(1) = Vector6d(w2(0), w2(1), w2(2), v2(0), v2(1), v2(2));
    // S_axis.col(2) = Vector6d(w3(0), w3(1), w3(2), v3(0), v3(1), v3(2));
    S_axis.col(0) << w1, v1;
    S_axis.col(1) << w2, v2;
    S_axis.col(2) << w3, v3;
}

void Leg::set_BodyScrew_Axis_List() {
    // HOme Configuration (M)상태에서 End-Effector frame 기준으로 본 Screw Axis 
    // Define the screw axes for the leg joints
    Eigen::Matrix4d M_inv = M_home.inverse();
    for (int i = 0; i < 3; i++) {
        B_axis.col(i) = adjointTransform(M_inv, S_axis.col(i));
    }
    // std::cout << "Resulting Body Screw Axis List:\n" << B_axis << std::endl;
}

// Compute the 3x3 skew-symmetric matrix of a 3x1 omega vector for Matrix cross product computation
// 그냥 계산을 할때는 최적화가 되어있는 cross method가 나음
Eigen::Matrix3d Leg::skewSymmetric(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d skew;
    skew <<     0, -omega(2),  omega(1),
            omega(2),      0, -omega(0),
           -omega(1),  omega(0),      0;
    return skew;
}

// Adjoint Transformation (twist or srcew axis 변환 함수)
Vector6d Leg::adjointTransform(const Eigen::Matrix4d& T, const Vector6d& V) {
    // Matrix를 만들어 6x6 , 6x1 행렬곱 연산 보다 w, v 따로 계산하는 것이 효율적(메모리)
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d p = T.block<3,1>(0,3);
    Eigen::Vector3d w = V.head<3>();
    Eigen::Vector3d v = V.tail<3>();
    Vector6d ret;

    ret.head<3>() = R * w;
    ret.tail<3>() = p.cross(w) + R * v;
    return ret;
}

// Compute transformation matrix using the matrix exponential of a screw axis and joint angle
// Using Rodrigues' formula for rotation matrix computation 
// w = 1 or v =1 일 때 나눠서 처리
// 최적화를 위해 T_out 변수 추가하여 재사용
void Leg::exp_coordinate_Operator(const Vector6d& B, const double theta, Eigen:: Matrix4d& T_out) {
    T_out.setIdentity();

    Eigen::Vector3d v = B.tail<3>();

    if (std::abs(theta) < 1e-9) {
        T_out.block<3,1>(0,3) = v * theta;
        return;
    }
    
    Eigen::Vector3d w = B.head<3>();
    Eigen::Matrix3d bracket_omega = skewSymmetric(w);

    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    Eigen::Matrix3d R = T_out.block<3,3>(0,0) + sin_theta * bracket_omega
                        + (1 - cos_theta) * (bracket_omega * bracket_omega);

    Eigen::Vector3d G_theta_v = v * theta
                                + (1 - cos_theta) * (w.cross(v))
                                + (theta - sin_theta) * (w.cross(w.cross(v)));

    T_out.block<3,3>(0,0) = R;
    T_out.block<3,1>(0,3) = G_theta_v;

    //Transformation_Mat.block<3,3>(0,0) = Eigen::AngleAxisd(theta, w).toRotationMatrix();

    return;
}

// -------------------------------Forward Kinematics Function-----------------------------------

// Compute the forward kinematics using PoE formula (space frame) 포워드 키네매틱스 계산 함수
Eigen::Matrix4d Leg::bodyframe_PoE(const Eigen::Vector3d& joint_angles){
    Eigen::Matrix4d T_total = M_home;
    for (int i = 0; i < 3; i++) {
        exp_coordinate_Operator(B_axis.col(i), joint_angles[i], T_temp);
        T_total = T_total * T_temp;
    };
    return T_total;
}

Eigen::Matrix4d Leg::spaceframe_PoE(const Eigen::Vector3d& joint_angles){
    Eigen::Matrix4d T_total = M_home;
    for (int i = 2; i >= 0; i--) {
        // spcae frame을 사용하고 있기에 base frame에서부터 곱해나가는게 직관적으로 맞을듯
        // adjoint Transformation으로도 가능하다는데 이후 고민해봐야할 듯
        exp_coordinate_Operator(S_axis.col(i), joint_angles[i], T_temp);
        T_total = T_temp * T_total;
    };
    return T_total;
}

// Wrapper function to perform FK solving and print the result 인터페이스 함수
void Leg::FK_solver() {
    setJoint_Angle_Vector();
    
    Eigen::Matrix4d result = bodyframe_PoE(JOINT_ANG);
    //Eigen::Matrix4d result = spaceframe_PoE(joint_angles);
    std::cout << "Resulting Transformation Matrix:\n" << result << std::endl;

    // 디버깅 함수 
    // joint_angles << 0.0, 0.7854, -1.5708; 
    // std::cout << "--- Debug FK Steps ---" << std::endl;
    // // 1. 초기값 M 확인
    // Eigen::Matrix4d T = M_home;
    // std::cout << "Step 0 (Home M):\n" << T << "\n\n";
    // // 2. 각 관절 적용 후 변화 확인
    // for (int i = 0; i < 3; i++) {
    //     // 현재 적용하는 Screw Axis와 각도 출력
    //     std::cout << "[Joint " << i << "] Angle: " << joint_angles[i] << std::endl;
    //     std::cout << "B_axis col: " << B_axis.col(i).transpose() << std::endl;
    //     Eigen::Matrix4d T_exp = exp_coordinate_Operator(B_axis.col(i), joint_angles[i]);     
    //     // 지수맵 결과 행렬 출력 (여기서 회전/이동이 제대로 생기는지 확인)
    //     std::cout << "Exp Matrix:\n" << T_exp << "\n";    
    //     T = T * T_exp; 
    //     std::cout << "Result after Joint " << i << ":\n" << T << "\n-----------------\n";
    //}
}

// -------------------------------Inverse Kinematics Function-----------------------------------
// Eigen::Vector3d Leg::bodyframe_NR(const Eigen::Matrix4d& T_d, const Eigen::Vector3d& initial_guess) {
    
//     return 
// }


Eigen::Vector3d Leg::bodyframe_ANL(const Eigen::Matrix4d& T_d) {
    // 1. Workspace Validity Check & Early Exit
    // 목표 위치 추출 (Target Position)
    Eigen::Vector3d position = T_d.block<3,1>(0,3);
    double x = position[0];
    double y = position[1];
    double z = position[2];

    // YZ 평면 투영 거리 (HAA 관절 회전 평면까지의 거리 계산)
    double yz_dist_sq = y*y + z*z;
    double yz_dist = std::sqrt(yz_dist_sq);

    // [Singularity Check 1] 목표점이 HAA 오프셋 원 안쪽에 있으면 도달 불가 (물리적 한계)
    if (yz_dist < HAA_Offset) {
        std::cerr << "[IK Error] Target inside HAA offset deadzone." << std::endl;
        return JOINT_ANG; // 현재 상태 유지
    }

    // 다리 평면상에서의 높이 (h) 및 힙(HFE)에서 목표점까지의 거리 (r)
    double h = std::sqrt(yz_dist_sq - HAA_Offset*HAA_Offset); 
    double r_sq = x*x + h*h;
    double r = std::sqrt(r_sq);

    // [Workspace Check] 최대/최소 도달 거리 확인
    double max_reach = Upper_Link + Lower_Link;
    double min_reach = std::abs(Upper_Link - Lower_Link); // 다리가 완전히 접혔을 때의 최소 거리

    if (r > max_reach) {
        std::cerr << "[IK Error] Target out of reach (too far)." << std::endl;
        return JOINT_ANG;
    }
    if (r < min_reach) {
        std::cerr << "[IK Error] Target too close (self-collision risk)." << std::endl;
        return JOINT_ANG;
    }

    // 2. Analytical Solution Calculation
    // q0 (HAA): 기하학적으로 결정 (Righty/Lefty configuration에 따라 부호가 바뀔 수 있으나, 여기선 표준 설정 사용)
    double q0 = atan2(y, -z) - atan2(HAA_Offset, h);

    // pre-check range of q0
    bool valid_0 = (q0 >= JOINT_LIMITS[0][0] && q0 <= JOINT_LIMITS[0][1]);
    
    if (!valid_0) {
        std::cerr << "[IK Error] HAA angle solution violates joint limits." << std::endl;
        return JOINT_ANG; // 현재 상태 유지
    }

    // Law of Cosines (제2 코사인 법칙)
    double cos_alpha = (r_sq + Upper_Link*Upper_Link - Lower_Link*Lower_Link) / (2 * r * Upper_Link);
    double cos_beta  = (Upper_Link*Upper_Link + Lower_Link*Lower_Link - r_sq) / (2 * Upper_Link * Lower_Link);

    // 수치 오차로 인한 범위 초과 방지 (Clamp)
    cos_alpha = std::max(-1.0, std::min(1.0, cos_alpha));
    cos_beta  = std::max(-1.0, std::min(1.0, cos_beta));

    double alpha = acos(cos_alpha); // HFE에서의 내부 각도
    double beta  = acos(cos_beta);  // KFE에서의 내부 각도 (무릎 각도)
    double gamma = atan2(-x, h);    // 목표 벡터의 각도

    // 3. Select Solution (Two possibilities for Knee: Forward vs Backward)
    // Solution A: Knee Forward (Standard, ' > ' shape) -> 무릎이 양의 방향으로 굽혀짐 (설정에 따라 다름)
    double q1_a = gamma - alpha;
    double q2_a = pi - beta; 

    // Solution B: Knee Backward (Flipped, ' < ' shape) -> 무릎이 음의 방향으로 굽혀짐
    double q1_b = gamma + alpha;
    double q2_b = beta - pi; 

    // 4. Joint Limit Check & Optimization (Minimum Displacement)
    // 각 해가 관절 제한 범위 내에 있는지 확인
    bool valid_a = (q1_a >= JOINT_LIMITS[1][0] && q1_a <= JOINT_LIMITS[1][1]) &&
                   (q2_a >= JOINT_LIMITS[2][0] && q2_a <= JOINT_LIMITS[2][1]);
    
    bool valid_b = (q1_b >= JOINT_LIMITS[1][0] && q1_b <= JOINT_LIMITS[1][1]) &&
                   (q2_b >= JOINT_LIMITS[2][0] && q2_b <= JOINT_LIMITS[2][1]);

    Eigen::Vector3d final_sol;

    // 이전 상태(JOINT_ANG)와 가장 가까운 해를 선택 (Smooth Motion)
    double dist_a = std::pow(JOINT_ANG[1] - q1_a, 2) + std::pow(JOINT_ANG[2] - q2_a, 2);
    double dist_b = std::pow(JOINT_ANG[1] - q1_b, 2) + std::pow(JOINT_ANG[2] - q2_b, 2);

    if (valid_a && valid_b) {
        // 둘 다 가능하면 움직임이 적은 쪽 선택
        if (dist_a <= dist_b) {
            final_sol = Eigen::Vector3d(q0, q1_a, q2_a);
        } else {
            final_sol = Eigen::Vector3d(q0, q1_b, q2_b);
        }
    } else if (valid_a) {
        final_sol = Eigen::Vector3d(q0, q1_a, q2_a);
    } else if (valid_b) {
        final_sol = Eigen::Vector3d(q0, q1_b, q2_b);
    } else {
        std::cerr << "[IK Error] Solutions exist but violate joint limits." << std::endl;
        return JOINT_ANG; // 모든 해가 제한 범위를 벗어남
    }
    JOINT_ANG = final_sol; // 상태 업데이트

    return final_sol;
}

// void Leg::IK_solver() {

// }

// // --------------------------------error checking/handling funtion-----------------------------------

// bool checkWorkspace(const Eigen::Matrix4d& T_d) {
//     Eigen::Vector3d position = T_d.block<3,1>(0,3);
    
//     double x = position[0];
//     double y = position[1];
//     double z = position[2];

//     // 1. yz 평면에서의 거리 (Roll 축 기준 거리)
//     double d_yz_sq = y*y + z*z;
//     double d_off_sq = HAA_Offset * HAA_Offset; // D_OFF = 0.04

//     // [중요] 목표점이 힙 오프셋 원 안쪽에 있으면 도달 불가
//     if (d_yz_sq < d_off_sq) return false;

//     // 2. 평면 투영 거리 계산 (Effective Length D)
//     // x축(Pitch 전후)과 yz 평면에서 오프셋을 뺀 나머지 거리의 합성
//     double D_sq = x*x + (d_yz_sq - d_off_sq);
//     double D = std::sqrt(D_sq);

//     // 3. 물리적 한계 체크 (L1=0.21, L2=0.19)
//     double L1 = Upper_Link;
//     double L2 = Lower_LInk;
    
//     // 최대 도달 거리 (다리를 폈을 때)
//     if (D > (L1 + L2)) return false;

//     // 최소 도달 거리 (무릎이 최대치인 -2.705 rad까지 접혔을 때)
//     // d^2 = L1^2 + L2^2 - 2*L1*L2*cos(pi - |theta_kfe|)
//     double min_dist = std::sqrt(L1*L1 + L2*L2 - 2*L1*L2*std::cos(pi - joint_limits[2][0]));
//     if (D < min_dist) return false;

//     return true; 
// }

bool Leg::checkJointLimits(const Eigen::Vector3d& joint_angles) {
    for (size_t i = 0; i < joint_angles.size(); i++) {
        if (joint_angles[i] < JOINT_LIMITS[i][0] || joint_angles[i] > JOINT_LIMITS[i][1]) {
            return false;
        }
    }
    return true;
}

bool Leg::checkSingularity(const Eigen::Matrix<double, 6, 3>& J) {
    // 자코비안 행렬의 최소 특이값이 특정 임계값보다 작은지 확인하여 특이점 여부 판단
    Eigen::JacobiSVD<Eigen::Matrix<double, 6, 3>> svd(J);
    double min_singular_value = svd.singularValues().minCoeff();
    const double singularity_threshold = 1e-6; // 임계값 설정 (조정 필요)
    return min_singular_value < singularity_threshold;
}

// --------------------------------테스트 함수-----------------------------------

// Set joint angles from user input 테스트를 위한 입력함수
void Leg::setJoint_Angle_Vector() {
    for (size_t i = 0; i < JOINT_ANG.size(); i++) {
        std::cin >> JOINT_ANG[i];
    }
}

Eigen::Vector3d Leg::setJoint_Angle_Vector_ret() {
    Eigen::Vector3d ret;
    for (size_t i = 0; i < JOINT_ANG.size(); i++) {
        std::cin >> ret[i];
    }
    return ret;
}