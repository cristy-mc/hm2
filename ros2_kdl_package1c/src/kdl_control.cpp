#include "kdl_control.h"
#include "utils.h" // Assicurati che contenga pseudoinverse()
#include <iostream>

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

// -----------------------------------------------------------------
// HELPER FUNCTIONS (Definite una volta sola!)
// -----------------------------------------------------------------

// Converte KDL::Rotation in Eigen::Matrix3d
Eigen::Matrix3d KDLController::toEigen(const KDL::Rotation& r) {
    Eigen::Matrix3d m;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            m(i, j) = r(i, j);
    return m;
}

// -----------------------------------------------------------------
// CONTROLLORI STANDARD
// -----------------------------------------------------------------

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;
    Eigen::VectorXd ddqd = _ddqd.data;
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis() + robot_->getGravity();
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    return Eigen::VectorXd::Zero(robot_->getNrJnts());
}

KDL::JntArray KDLController::velocity_ctrl(const Eigen::VectorXd& cartvel)
{
    Eigen::MatrixXd J = robot_->getEEJacobian().data;
    Eigen::MatrixXd J_pinv = pseudoinverse(J);
    KDL::JntArray q_dot_cmd;
    q_dot_cmd.resize(robot_->getNrJnts());
    q_dot_cmd.data = J_pinv * cartvel; 
    return q_dot_cmd;
}

// -----------------------------------------------------------------
// VISION CONTROL (Visual Servoing)
// -----------------------------------------------------------------

Eigen::VectorXd KDLController::velocity_ctrl_vision(
    const Eigen::Vector3d &p_o_c,    
    const KDL::JntArray &q_curr,
    double K_gain)
{
    // (void)q_curr; // q_curr serve per l'update se non fatto fuori, ma qui usiamo robot_->getEEFrame()
    
    // 1. Adattamento Frame Aruco -> Camera
    // Matrice che inverte gli assi X e Y (come nel codice del tuo amico)
    Eigen::Matrix3d R_image_to_camera;
    R_image_to_camera << -1,  0,  0,
                          0, -1,  0,
                          0,  0,  1;

    Eigen::Vector3d s_cam = R_image_to_camera * p_o_c;

    // 2. Calcolo distanza e feature s normalizzata
    double dist = s_cam.norm();
    if (dist < 1e-6) dist = 1e-6;

    Eigen::Vector3d s = s_cam / dist;    
    Eigen::Vector3d s_d(0.0, 0.0, 1.0); // Target: asse Z ottico

    // 3. Matrice di Interazione L (Interaction Matrix)
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d skew_s;
    skew_s <<   0, -s(2),  s(1),
              s(2),    0, -s(0),
             -s(1),  s(0),    0;

    Eigen::MatrixXd L(3,6);
    // L = -1/dist * (I - s*s^T)  |  skew(s)
    L.block<3,3>(0,0) = -(I - s * s.transpose()) / dist;
    L.block<3,3>(0,3) = skew_s;

    // 4. Calcolo Jacobiana Ruotata (Cruciale per il tuo robot)
    
    // Rotazione Base -> EE
    KDL::Frame base_T_ee = robot_->getEEFrame();
    Eigen::Matrix3d R_base_ee = toEigen(base_T_ee.M);

    // Rotazione EE -> Camera (Statica da URDF: 0, -1.57, 3.14)
    KDL::Rotation kdl_R_ee_cam = KDL::Rotation::RPY(0.0, -1.57, 3.14);
    Eigen::Matrix3d R_ee_cam = toEigen(kdl_R_ee_cam);

    // Rotazione Totale Base -> Camera
    Eigen::Matrix3d R_base_cam = R_base_ee * R_ee_cam;

    // Jacobiana Base
    Eigen::MatrixXd J_ee = robot_->getEEJacobian().data;
    int n_jnts = robot_->getNrJnts();
    
    // Ruotiamo la Jacobiana nel frame Camera: J_c = [R^T 0; 0 R^T] * J_ee
    Eigen::MatrixXd J_c(6, n_jnts);
    Eigen::Matrix3d R_cam_base = R_base_cam.transpose();
    
    J_c.block(0, 0, 3, n_jnts) = R_cam_base * J_ee.block(0, 0, 3, n_jnts);
    J_c.block(3, 0, 3, n_jnts) = R_cam_base * J_ee.block(3, 0, 3, n_jnts);

    // 5. Calcolo Jacobiana Task e Inversa
    Eigen::MatrixXd LJ = L * J_c; // (3xN)
    Eigen::MatrixXd LJ_pinv = pseudoinverse(LJ);
    
    // Spazio Nullo (per smorzamento)
    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(n_jnts, n_jnts) - LJ_pinv * LJ;

    Eigen::Vector3d s_error = s_d - s;

    // 6. Legge di Controllo Finale
    // q_dot = Gain * J_pseudo * error
    Eigen::VectorXd q_dot = K_gain * (LJ_pinv * s_error);

    // Smorzamento nullo (opzionale, qui settato a zero)
    Eigen::VectorXd q_dot0 = Eigen::VectorXd::Zero(n_jnts);
    q_dot += N * q_dot0;

    // Debug (decommentare se serve)
    // std::cout << "s_error: " << s_error.transpose() << " | dist: " << dist << std::endl;

    return q_dot;
}