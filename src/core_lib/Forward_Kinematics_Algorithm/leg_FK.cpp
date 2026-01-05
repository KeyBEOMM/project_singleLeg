#include "leg_FK.h"

leg_FK::leg_FK() {
    // Initialize the home matrix
    set_Home_Matrix();
    T_theta = M_home;
    // Initialize the screw axis list
    set_Screw_Axis_List();
}

void leg_FK::set_Home_Matrix() {
    M_home = Eigen::Matrix4d::Identity();
    M_home(1, 3) = HAA_Offset;
    M_home(2, 3) = Upper_Link + Lower_Link;
}

Eigen::Matrix3d leg_FK::skewSymmetric(const Eigen::Vector3d& omega) {
    Eigen::Matrix3d skew;
    skew <<     0, -omega(2),  omega(1),
            omega(2),      0, -omega(0),
           -omega(1),  omega(0),      0;
    return skew;
}

Eigen::Matrix4d leg_FK::exp_coordinate_Operator(const Vector6d& S, const double theta) {
    Eigen::Matrix4d Transformation_Mat = Eigen::Matrix4d::Zero();

    Eigen::Vector3d bracket_omega = skewSymmetric(S.head<3>());
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

void leg_FK::set_Screw_Axis_List() {
    // Define the screw axes for the leg joints
    // Q. is it better to set parameters as variables and assign these values for other functions's result?
    S_axis_List.clear();
    S_axis_List.push_back(Vector6d(1, 0, 0, 0, 0, 0));
    S_axis_List.push_back(Vector6d(0, 1, 0, 0, 0, 0));
    S_axis_List.push_back(Vector6d(0, 1, 0, -0.21, 0, 0));
}

Eigen::Matrix4d leg_FK::singleLeg_FK(const std::vector<double>& joint_angles){
    for (int i = joint_angles.size(); i > 0; i--) {
        T_theta = M_home;
        Eigen::Matrix4d T = exp_coordinate_Operator(S_axis_List[i], joint_angles[i]);
        T_theta = T * T_theta;
    }
}

void leg_FK::Interface_function() {
    setJoint_Angle_Vector();
    
    Eigen::Matrix4d result = singleLeg_FK(joint_angle_vector);
    std::cout << "Resulting Transformation Matrix:\n" << result << std::endl;
}

void leg_FK::setJoint_Angle_Vector() {
    for (size_t i = 0; i < joint_angle_vector.size(); i++) {
        std::cin >> joint_angle_vector[i];
    }
}