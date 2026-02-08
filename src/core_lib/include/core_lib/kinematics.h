#include <iostream>
#include <Eigen/Dense>
#include <vector>

using Vector6d = Eigen::Matrix<double, 6, 1>;

class Leg {

public:
    // home matrix, Screw Axis, Matrix initialize
    Leg();
    ~Leg();

    void set_Home_Matrix();
    void set_Screw_Axis_List();
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& omega);
    Eigen::Matrix4d exp_coordinate_Operator(const Vector6d& S, const double theta);

    Eigen::Matrix4d spaceframe_PoE(const std::vector<double>& joint_angles);
    void FK_solver();
    
    
    void IK_solver();

    void setJoint_Angle_Vector();

private:
    Eigen::Matrix4d M_home;
    std::vector<Vector6d> S_axis_List;
    Eigen::Matrix4d T_theta;

    const double HAA_Offset = 0.04;
    const double Upper_Link = 0.21;
    const double Lower_Link = 0.19;   

    std::vector<double> joint_angle_vector = {0.0, 0.0, 0.0};
};