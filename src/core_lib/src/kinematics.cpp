#include "core_lib/kinematics.h"

/*
고민해볼 점
0. 동적 메모리 할당 최소화 -> Eigen::Matrix4d Transformation_Mat = Eigen::Matrix4d::Zero(); 등 불필요한 할당 제거 + std::vector x
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
    M_home = Eigen::Matrix4d::Identity();
    M_home(1, 3) = HAA_Offset;
    M_home(2, 3) = -(Upper_Link + Lower_Link);
}

// Set the screw axis list for the leg joints 현재 하드코딩 되어있는데 이후 config 파일로부터 불러오도록 변경 필요
void Leg::set_Screw_Axis_List() {
    // Define the screw axes for the leg joints
    // Q. is it better to set parameters as variables and assign these values for other functions's result?
    S_axis.col(0) = Vector6d(1, 0, 0, 0, 0, 0);
    S_axis.col(1) = Vector6d(0, 1, 0, 0, 0, 0);
    S_axis.col(2) = Vector6d(0, 1, 0, 0.21, 0, 0);
}

void Leg::set_BodyScrew_Axis_List() {
    // Define the screw axes for the leg joints
    // Q. is it better to set parameters as variables and assign these values for other functions's result?
    Eigen::Matrix4d M_inv = M_home.inverse();
    for (int i = 0; i < 3; i++) {
        B_axis.col(i) = adjointTransform(M_inv, S_axis.col(i));
    }
    B_axis.col(0) = Vector6d(1, 0, 0, 0, -0.4, 0);
    B_axis.col(1) = Vector6d(0, 1, 0, 0, 0, 0);
    B_axis.col(2) = Vector6d(0, 1, 0, 0.21, 0, 0);
}

// Compute the 3x3 skew-symmetric matrix of a 3x1 omega vector for Matrix cross product computation
Eigen::Matrix3d Leg::skewSymmetric(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d skew;
    skew <<     0, -omega(2),  omega(1),
            omega(2),      0, -omega(0),
           -omega(1),  omega(0),      0;
    return skew;
}

// Adjoint Transformation (twist or srcew axis 변환 함수)
Eigen::Matrix<double, 6, 1> Leg::adjointTransform(const Eigen::Matrix4d& T, const Vector6d& V) {
    // Matrix를 만들어 6x6 , 6x1 행렬곱 연산 보다 w, v 따로 계산하는 것이 효율적(메모리)
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d p = T.block<3,1>(0,3);

    Vector6d ret;

    ret.head<3>() = R * V.head<3>();
    ret.tail<3>() = p.cross(ret.head<3>()) + R * V.tail<3>();
    return ret;
}

// Compute transformation matrix using the matrix exponential of a screw axis and joint angle
// Using Rodrigues' formula for rotation matrix computation 
// w = 1 or v =1 일 때 나눠서 처리
Eigen::Matrix4d Leg::exp_coordinate_Operator(const Vector6d& B, const double theta) {
    Eigen::Matrix4d Transformation_Mat = Eigen::Matrix4d::Zero();
    Eigen::Vector3d v = B.tail<3>();

    if (theta < 1e-9) {
        Transformation_Mat = Eigen::Matrix4d::Identity();
        Transformation_Mat.block<3,1>(0,3) = v * theta;
        return Transformation_Mat;
    }
    
    Eigen::Vector3d w = B.head<3>();
    Eigen::Matrix3d bracket_omega = skewSymmetric(w);

    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + sin_theta * bracket_omega
                        + (1 - cos_theta) * (bracket_omega * bracket_omega);

    Eigen::Vector3d G_theta_v = v * theta
                                + (1 - cos_theta) * (w.cross(v))
                                + (theta - sin_theta) * (w.cross(w.cross(v)));

    Transformation_Mat.block<3,3>(0,0) = R;
    Transformation_Mat.block<3,1>(0,3) = G_theta_v;
    Transformation_Mat(3,3) = 1.0;

    return Transformation_Mat;
}

// -------------------------------Forward Kinematics Function-----------------------------------

// Compute the forward kinematics using PoE formula (space frame) 포워드 키네매틱스 계산 함수
Eigen::Matrix4d Leg::bodyframe_PoE(const Eigen::Vector3d& joint_angles){
    T_theta = M_home;
    for (int i = 0; i < joint_angles.size(); i++) {
        // spcae frame을 사용하고 있기에 base frame에서부터 곱해나가는게 직관적으로 맞을듯
        // adjoint Transformation으로도 가능하다는데 이후 고민해봐야할 듯
        Eigen::Matrix4d T = exp_coordinate_Operator(B_axis.col(i), joint_angles[i]);
        T_theta = T_theta * T;
    };
    return T_theta;
}

// Wrapper function to perform FK solving and print the result 인터페이스 함수
void Leg::FK_solver() {
    setJoint_Angle_Vector();
    
    Eigen::Matrix4d result = bodyframe_PoE(joint_angles);
    std::cout << "Resulting Transformation Matrix:\n" << result << std::endl;
}

// -------------------------------Inverse Kinematics Function-----------------------------------

void Leg::IK_solver() {
    std::cout << "Inverse Kinematics solver is not yet implemented." << std::endl;
}



// --------------------------------테스트 함수-----------------------------------

// Set joint angles from user input 테스트를 위한 입력함수
void Leg::setJoint_Angle_Vector() {
    for (size_t i = 0; i < joint_angles.size(); i++) {
        std::cin >> joint_angles[i];
    }
}