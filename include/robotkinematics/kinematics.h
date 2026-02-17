#pragma once
#include <cmath>
#include<Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <vector>

namespace Kin {
using Vector6d = Eigen::Matrix<double, 6, 1>;


struct DH_Param{
    double a;
    double alpha;
    double d;
    double theta;
};

class Kinematics{

    public:
    Kinematics(const int dof, const bool tool);
    ~Kinematics()=default;
    void setDHParam(const std::vector<DH_Param>& dh_param);
    Eigen::Matrix4d forwardKinematics(const Eigen::VectorXd& q);
    Eigen::VectorXd inverseKinematics(const Eigen::Matrix4d& desired_pose, const Eigen::VectorXd& q_guess);
    Eigen::MatrixXd getTCPJacobian(const Eigen::VectorXd& q);
    Eigen::Vector3d getTCPVelocity(const Eigen::VectorXd& qdot, const Eigen::VectorXd& q);
    Vector6d logSE3Map(const Eigen::Matrix4d& T);
    Eigen::MatrixXd getBodyJacobian(const Eigen::VectorXd& q);


    private:
    int dof_;
    bool tool_;
    std::vector<DH_Param> dh_params_;
     
};


}