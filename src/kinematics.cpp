#include <robotkinematics/kinematics.h>
#include<cassert>
namespace Kin { 

Kinematics::Kinematics(const int dof, const bool tool) : dof_(dof), tool_(tool) {
    dh_params_.resize(dof_);    }


void Kinematics::setDHParam(const std::vector<DH_Param>& dh_param) {
    assert(dh_param.size() == dof_ && "Size of DH parameters must match the degrees of freedom.");
    dh_params_ = dh_param;
}
static Eigen::Matrix3d skew(const Eigen::Vector3d& w) {
    Eigen::Matrix3d W;
    W <<     0, -w.z(),  w.y(),
          w.z(),     0, -w.x(),
         -w.y(),  w.x(),     0;
    return W;
}

static Eigen::Matrix4d dh(double a, double alpha, double d, double theta) {
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    const double ct = std::cos(theta), st = std::sin(theta);
    const double ca = std::cos(alpha), sa = std::sin(alpha);

    T(0,0)= ct;   T(0,1)= -st*ca;  T(0,2)=  st*sa;  T(0,3)= a*ct;
    T(1,0)= st;   T(1,1)=  ct*ca;  T(1,2)= -ct*sa;  T(1,3)= a*st;
    T(2,0)= 0.0;  T(2,1)=     sa;  T(2,2)=     ca;  T(2,3)=    d;
    return T;
}


Eigen::Matrix4d Kinematics::forwardKinematics(const Eigen::VectorXd& q) {
    assert(q.size() == dof_ && "Size of joint angles must match the degrees of freedom.");
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < dof_; ++i) {
        double a = dh_params_[i].a;
        double alpha = dh_params_[i].alpha;
        double d = dh_params_[i].d;
        double theta = dh_params_[i].theta + q(i);
        
        Eigen::Matrix4d A;
        A = dh(a, alpha, d, theta);        
        T *= A;
    }
    return T;
}

Eigen::MatrixXd Kinematics::getTCPJacobian(const Eigen::VectorXd& q) {
    Eigen::MatrixXd J(6, dof_);
    J.setZero();

    // Store T0i for i = 0..dof_ (T0_0 = I, T0_dof_ = T0ee)
    std::vector<Eigen::Matrix4d> T0(dof_ + 1);
    T0[0] = Eigen::Matrix4d::Identity();

    // Forward chain 
    for (int i = 0; i < dof_; ++i) {
        const double theta = q(i) + dh_params_[i].theta;
        T0[i+1] = T0[i] * dh(dh_params_[i].a, dh_params_[i].alpha, dh_params_[i].d, theta);
    }

    const Eigen::Vector3d o_ee = T0[dof_].block<3,1>(0,3);

    for (int i = 0; i < dof_; ++i) {
        const Eigen::Vector3d o_i = T0[i].block<3,1>(0,3);
        const Eigen::Vector3d z_i = T0[i].block<3,1>(0,2); // z-axis of frame i in base coords

        // For revolute joints: linear part = z_i x (o_ee - o_i), angular part = z_i
            J.block<3,1>(0,i) = z_i.cross(o_ee - o_i); // linear
            J.block<3,1>(3,i) = z_i;                  // angular
 
    }

    return J; // This is the SPACE (base-frame) geometric Jacobian
}

Eigen::MatrixXd Kinematics::getBodyJacobian(const Eigen::VectorXd& q) {
    const Eigen::MatrixXd Js = getTCPJacobian(q);      // 6 x n, space Jacobian, [v; w] in base frame
    const Eigen::Matrix4d T  = forwardKinematics(q);   // T0e
    const Eigen::Matrix3d R  = T.block<3,3>(0,0);
    const Eigen::Vector3d p  = T.block<3,1>(0,3);

    const Eigen::Matrix3d Rt = R.transpose();
    const Eigen::Matrix3d px = skew(p);

    Eigen::MatrixXd Jb(6, dof_);
    const Eigen::MatrixXd Jv = Js.topRows(3);
    const Eigen::MatrixXd Jw = Js.bottomRows(3);

    // Jb = Ad_{T^{-1}} Js
    Jb.topRows(3)    = Rt * (Jv - px * Jw);
    Jb.bottomRows(3) = Rt * Jw;

    return Jb;
}


Eigen::Vector3d Kinematics::getTCPVelocity(const Eigen::VectorXd& qdot, const Eigen::VectorXd& q) {
    // Placeholder: Compute the TCP velocity using the Jacobian and joint velocities.
    // v_tcp = J * q_dot
    Eigen::MatrixXd J = getTCPJacobian(q);
    return J.topRows(3) * qdot; // Linear velocity part
}


Vector6d Kinematics::logSE3Map(const Eigen::Matrix4d& T) {
    const Eigen::Matrix3d R = T.block<3,3>(0,0);
    const Eigen::Vector3d p = T.block<3,1>(0,3);

    // --- so(3) log ---
    double cos_theta = (R.trace() - 1.0) * 0.5;
    cos_theta = std::min(1.0, std::max(-1.0, cos_theta));
    const double theta = std::acos(cos_theta);

    Eigen::Vector3d w = Eigen::Vector3d::Zero();
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d V_inv = Eigen::Matrix3d::Identity();

    constexpr double eps = 1e-9;

    if (theta < eps) {
        // R ~ I : w ~ 0, use series for V_inv
        // For very small angles: V ≈ I + 0.5 W + ...
        // so V_inv ≈ I - 0.5 W + ...
        // Here W ~ 0 anyway
        V_inv = Eigen::Matrix3d::Identity();
        w.setZero();
    } else {
        // w_hat (unit axis) from (R - R^T)
        Eigen::Vector3d w_hat;
        w_hat << (R(2,1) - R(1,2)),
                 (R(0,2) - R(2,0)),
                 (R(1,0) - R(0,1));
        w_hat *= (1.0 / (2.0 * std::sin(theta)));

        // rotation vector (axis * angle)
        w = theta * w_hat;
        W = skew(w);

        // V_inv formula ( Murray-Li-Sastry "Mathematical Introduction to Robotic Manipulation", Section 3.2.3):
        // V_inv = I - 0.5*W + a*W^2
        // a = 1/theta^2 * (1 - (theta*sin(theta))/(2*(1 - cos(theta))))
        const double sin_t = std::sin(theta);
        const double one_minus_cos = 1.0 - std::cos(theta);

        const double a = (1.0/(theta*theta)) * (1.0 - (theta * sin_t)/(2.0 * one_minus_cos));
        V_inv = Eigen::Matrix3d::Identity() - 0.5 * W + a * (W * W);
    }

    const Eigen::Vector3d v = V_inv * p;

    Vector6d xi;
    xi << v, w;
    return xi;
}

Eigen::VectorXd Kinematics::inverseKinematics(
    const Eigen::Matrix4d& desired_pose,
    const Eigen::VectorXd& q_guess)
{
    // --- Damped Least Squares IK --- // (body frame error, so Jacobian must be body-frame)
    Eigen::VectorXd q = q_guess;
    const int max_iters = 1000;
    const double tol = 1e-8;
    const double lambda = 1e-5;   // damping
    const double step = 0.15;      // change this to < 1.0 for more conservative updates

    for (int iter = 0; iter < max_iters; ++iter) {
        const Eigen::Matrix4d T = forwardKinematics(q);

        // Body-frame error: T_err = T^{-1} * T_des
        const Eigen::Matrix4d T_err = T.inverse() * desired_pose;

        Vector6d xi = logSE3Map(T_err);  // [v; w] body twist taking current -> desired

        if (xi.norm() < tol) {
            return q;
        }

        // Recompute body Jacobian at current q 
        Eigen::MatrixXd J = getBodyJacobian(q); // (6 x dof_)

        // Damped least squares: dq = (J^T J + λ^2 I)^{-1} J^T xi
        Eigen::MatrixXd A = J.transpose() * J + (lambda * lambda) * Eigen::MatrixXd::Identity(dof_, dof_);
        Eigen::VectorXd b = J.transpose() * xi;
        Eigen::VectorXd dq = A.ldlt().solve(b);

        q += step * dq;
    }

    return q;
}

}