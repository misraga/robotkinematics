#include<iostream>
#include "robotkinematics/kinematics.h"


int main() {
    // Define DH parameters for the Viper650 robot
    int dof = 6;
    std::vector<Kin::DH_Param> dh_params(dof);
    // VIper 650 DH parameters 
    dh_params[0].alpha = 0;
    dh_params[0].a = 0.0;
    dh_params[0].d = 0.0;   
    dh_params[0].theta = 0.0; // This will be added to the joint angle q(0)

    dh_params[1].alpha = -M_PI/2;
    dh_params[1].a = 75/1000.0; // Convert mm to meters
    dh_params[1].d = 0.0;
    dh_params[1].theta = 0.0;

    dh_params[2].alpha = 0.0;
    dh_params[2].a = 270.0/1000.0; // Convert mm to meters
    dh_params[2].d = 0.0;
    dh_params[2].theta = 0.0;

    dh_params[3].alpha = M_PI/2;
    dh_params[3].a = 0.0;
    dh_params[3].d = 295/1000.0; // Convert mm to meters
    dh_params[3].theta = 0.0;

    dh_params[4].alpha = -M_PI/2;
    dh_params[4].a = 0.0;
    dh_params[4].d = 0.0;
    dh_params[4].theta = 0.0;

    dh_params[5].alpha = 0.0;
    dh_params[5].a = 0.0;
    dh_params[5].d = 80.0/1000.0; // Convert mm to meters
    dh_params[5].theta = 0.0;

    Kin::Kinematics viper_kinematics(dof, true);
    viper_kinematics.setDHParam(dh_params);

    // Example joint angles (in radians)
    Eigen::VectorXd q(6);
    q << M_PI/6, -M_PI/4, M_PI/3, -M_PI/6, M_PI/4, -M_PI/3;

    // Compute forward kinematics
    Eigen::Matrix4d end_effector_pose = viper_kinematics.forwardKinematics(q);
    std::cout << "End Effector Pose:\n" << end_effector_pose << std::endl;

    Eigen::Matrix4d desired_pose = end_effector_pose; // For testing, we can use the same pose
    Eigen::VectorXd q_guess = q-Eigen::VectorXd::Random(dof)*0.1 ; // Start IK with perturbation from the original joint angles
    Eigen::VectorXd q_solution = viper_kinematics.inverseKinematics(desired_pose, q_guess);
    std::cout << "IK Solution:\n" << q_solution.transpose() << std::endl;   
    std::cout << "IK initial guess:\n" << q_guess.transpose() << std::endl;   

    return 0;
}