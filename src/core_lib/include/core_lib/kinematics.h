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
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // home matrix, Screw Axis, Matrix initialize
    Leg();
    ~Leg();

    // seter functions
    void set_Home_Matrix();
    void set_Screw_Axis_List();
    void set_BodyScrew_Axis_List();
    void setJoint_Angle_Vector();
    Eigen::Vector3d setJoint_Angle_Vector_ret();

    // helper functions
    Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& omega); // bracket operation이 외적을 대체하는 것은 맞지만, 라이브러리 메서드 사용이 더 효율적임. 하지만 행렬 미분을 해야할 경우 필요
    void exp_coordinate_Operator(const Vector6d& B, const double theta, Eigen::Matrix4d& T_out);
    Eigen::Matrix<double, 6, 1> adjointTransform(const Eigen::Matrix4d& T, const Vector6d& V);
    
    // FK solver functions
    Eigen::Matrix4d bodyframe_PoE(const Eigen::Vector3d& joint_angles);
    Eigen::Matrix4d spaceframe_PoE(const Eigen::Vector3d& joint_angles);
    void FK_solver();
    
    // IK solver functions
    Eigen::Vector3d bodyframe_NR(const Eigen::Matrix4d& T_d, const Eigen::Vector3d& initial_guess);
    Eigen::Vector3d bodyframe_ANL(const Eigen::Matrix4d& T_d);
    Eigen::Vector3d bodyframe_DLS(const Eigen::Matrix4d& T_d);

    void IK_solver();
    // helper function

    // error checking/handling functions
    bool checkWorkspace(const Eigen::Matrix4d& T_d);
    bool checkJointLimits(const Eigen::Vector3d& joint_angles);
    bool checkSingularity(const Eigen::Matrix<double, 6, 3>& J);
    bool checkManipulability(const Eigen::Matrix<double, 6, 3>& J);

private:
    Eigen::Matrix4d M_home;
    Eigen::Matrix<double, 6, 3> B_axis;
    Eigen::Matrix<double, 6, 3> S_axis;
    Eigen::Matrix4d T_theta;
    Eigen::Vector3d JOINT_ANG = {0.0, 0.0, 0.0};

    Eigen::Vector3d INITIAL_ANL = {0.0, 0.7854, -1.5708};

    RobotState robot_state;

    const double HAA_Offset = 0.04;
    const double Upper_Link = 0.21;
    const double Lower_Link = 0.19;   

    double JOINT_LIMITS[3][2] = {
        {-0.7854, 2.0944},   // HAA joint limits in radians (-45 to 120 degrees) 반대 다리와의 각도합이 -90 이상일 떄는 더 넓은 범위를 허용해도 될듯,   // HAA joint limits in radians (-120 to 120 degrees) 
        {-1.5708, 2.7925},   // HFE joint limits in radians (-90 to 160 degrees)
        {-2.705, 2.705}       // KFE joint limits in radians (-155 to 155 degrees)
    };

//-----helper member variables-----//
    const double pi = 3.14159265358979323846;
    Eigen::Matrix4d T_temp;
    Eigen::Matrix3d R_temp;
};