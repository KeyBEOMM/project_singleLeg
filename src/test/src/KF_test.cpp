#include <iostream>
#include "core_lib/kinematics.h"

int main() {
    Leg* leg1 = new Leg();
    
    Eigen::Vector3d input_value = leg1->setJoint_Angle_Vector_ret();
    std::cout << "Input Joint Angles: " << input_value.transpose() << std::endl;

    Eigen::Matrix4d T_d = leg1->bodyframe_PoE(input_value);
    std::cout << "Desired Transformation Matrix (T_d):\n" << T_d << std::endl;

    Eigen::Vector3d joint_angles = leg1->bodyframe_ANL(T_d);
    std::cout << "Calculated Joint Angles (ANL): " << joint_angles.transpose() << std::endl;

    Eigen::Matrix4d T_d2 = leg1->bodyframe_PoE(joint_angles);
    std::cout << "Recomputed Transformation Matrix (T_d2):\n" << T_d2 << std::endl;

    delete leg1;
    return 0;   
}