#include "core_lib/kinematics.h"

"""고민해볼 점
1. 링크의 수가 변하면 사용 불가 -> 링크 수에 상관없이 동작하도록 변경 필요
2. home matrix와 screw axis list가 하드코딩 되어있음 -> config 파일로부터 불러오도록 변경 필요
3. 로드리게스 formula에서 cos, sin 계산이 중복 -> 미리 계산해두고 재사용하도록 변경 필요
4. error handling 없음 -> 추후 추가 필요
"""

Leg::Leg() {
    // Initialize the home matrix
    set_Home_Matrix();
    T_theta = M_home;
    // Initialize the screw axis list
    set_Screw_Axis_List();
}
Leg::~Leg() {
}

// Set the home configuration matrix M
void Leg::set_Home_Matrix() {
    M_home = Eigen::Matrix4d::Identity();
    M_home(1, 3) = HAA_Offset;
    M_home(2, 3) = -(Upper_Link + Lower_Link);
}

// Compute the 3x3 skew-symmetric matrix of a 3x1 omega vector for Matrix cross product computation
Eigen::Matrix3d Leg::skewSymmetric(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d skew;
    skew <<     0, -omega(2),  omega(1),
            omega(2),      0, -omega(0),
           -omega(1),  omega(0),      0;
    return skew;
}

// Compute transformation matrix using the matrix exponential of a screw axis and joint angle
// 현재 cos, sin 중복 계산 -> 미리 계산해두고 재사용하도록 변경 필요
Eigen::Matrix4d Leg::exp_coordinate_Operator(const Vector6d& S, const double theta) {
    Eigen::Matrix4d Transformation_Mat = Eigen::Matrix4d::Zero();

    Eigen::Matrix3d bracket_omega = skewSymmetric(S.head<3>());
    Eigen::Vector3d v = S.tail<3>();

    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() + sin(theta) * bracket_omega
                        + (1 - cos(theta)) * (bracket_omega * bracket_omega);

    Eigen::Vector3d G_theta_v = (Eigen::Matrix3d::Identity() * theta
                                + (1 - cos(theta)) * bracket_omega
                                + (theta - sin(theta)) * (bracket_omega * bracket_omega)) * v;

    Transformation_Mat.block<3,3>(0,0) = R;
    Transformation_Mat.block<3,1>(0,3) = G_theta_v;
    Transformation_Mat(3,3) = 1.0;

    return Transformation_Mat;
}

// Set the screw axis list for the leg joints 현재 하드코딩 되어있는데 이후 config 파일로부터 불러오도록 변경 필요
void Leg::set_Screw_Axis_List() {
    // Define the screw axes for the leg joints
    // Q. is it better to set parameters as variables and assign these values for other functions's result?
    S_axis_List.clear();
    S_axis_List.push_back(Vector6d(1, 0, 0, 0, 0, 0));
    S_axis_List.push_back(Vector6d(0, 1, 0, 0, 0, 0));
    S_axis_List.push_back(Vector6d(0, 1, 0, -0.21, 0, 0));
}

// -------------------------------Forward Kinematics Function-----------------------------------

// Compute the forward kinematics using PoE formula (space frame) 포워드 키네매틱스 계산 함수
Eigen::Matrix4d Leg::spaceframe_PoE(const std::vector<double>& joint_angles){
    T_theta = Eigen::Matrix4d::Identity();
    for (int i = 0; i < joint_angles.size(); i++) {
        // spcae frame을 사용하고 있기에 base frame에서부터 곱해나가는게 직관적으로 맞을듯
        // adjoint Transformation으로도 가능하다는데 이후 고민해봐야할 듯
        Eigen::Matrix4d T = exp_coordinate_Operator(S_axis_List[i], joint_angles[i]);
        T_theta = T_theta * T;
    }
    T_theta = T_theta * M_home;
    return T_theta;
}

// Wrapper function to perform FK solving and print the result 인터페이스 함수
void Leg::FK_solver() {
    setJoint_Angle_Vector();
    
    Eigen::Matrix4d result = spaceframe_PoE(joint_angle_vector);
    std::cout << "Resulting Transformation Matrix:\n" << result << std::endl;
}

// -------------------------------Inverse Kinematics Function-----------------------------------

void Leg::IK_solver() {
    std::cout << "Inverse Kinematics solver is not yet implemented." << std::endl;
}



// --------------------------------테스트 함수-----------------------------------

// Set joint angles from user input 테스트를 위한 입력함수
void Leg::setJoint_Angle_Vector() {
    for (size_t i = 0; i < joint_angle_vector.size(); i++) {
        std::cin >> joint_angle_vector[i];
    }
}