#include <iostream>
#include <Eigen/Dense>
#include <vector>

using Vector6d = Eigen::Matrix<double, 6, 1>;

struct RobotState
{
    Eigen::Matrix<double, 4, 4> T;
    Eigen::Matrix<double, 6, 3> Jb;
};

class Leg {

public:
    // home matrix, Screw Axis, Matrix initialize
    Leg();
    ~Leg();

    // seter functions
    void set_Home_Matrix();
    void set_Screw_Axis_List();
    void set_BodyScrew_Axis_List();
    void setJoint_Angle_Vector();

    // helper functions
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& omega); // bracket operation이 외적을 대체하는 것은 맞지만, 라이브러리 메서드 사용이 더 효율적임. 하지만 행렬 미분을 해야할 경우 필요
    Eigen::Matrix4d exp_coordinate_Operator(const Vector6d& B, const double theta);
    Eigen::Matrix<double, 6, 1> adjointTransform(const Eigen::Matrix4d& T, const Vector6d& V);
    
    // FK solver functions
    Eigen::Matrix4d bodyframe_PoE(const Eigen::Vector3d& joint_angles);
    void FK_solver();
    
    // IK solver functions
    void IK_solver();



private:
    Eigen::Matrix4d M_home;
    Eigen::Matrix<double, 6, 3> B_axis;
    Eigen::Matrix<double, 6, 3> S_axis;
    Eigen::Matrix4d T_theta;
    Eigen::Vector3d joint_angles = {0.0, 0.0, 0.0};

    RobotState robot_state;

    const double HAA_Offset = 0.04;
    const double Upper_Link = 0.21;
    const double Lower_Link = 0.19;   
};