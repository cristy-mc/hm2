#ifndef KDL_CONTROL_H
#define KDL_CONTROL_H

#include "kdl_robot.h" // Assumendo che questo sia l'header per KDLRobot
#include "utils.h"     // Per la funzione pseudoinverse
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
// #include <kdl/twist.hpp> // <-- RIMUOVI QUESTA RIGA
#include <Eigen/Dense>

class KDLController
{
public:
    KDLController(KDLRobot &_robot);

    // --- Funzioni Esistenti ---
    Eigen::VectorXd idCntr(KDL::JntArray &_qd,
                           KDL::JntArray &_dqd,
                           KDL::JntArray &_ddqd,
                           double _Kp, double _Kd);

    Eigen::VectorXd idCntr(KDL::Frame &_desPos,
                           KDL::Twist &_desVel,
                           KDL::Twist &_desAcc,
                           double _Kpp, double _Kpo,
                           double _Kdp, double _Kdo);

    KDL::JntArray velocity_ctrl(const Eigen::VectorXd& cartvel);

    // Firma modificata per accettare KDL::JntArray come nel nodo ROS
    KDL::JntArray velocity_ctrl_null(const Eigen::VectorXd& cartvel,
                                     double lambda,
                                     const KDL::JntArray& q_min,
                                     const KDL::JntArray& q_max);



Eigen::VectorXd velocity_ctrl_vision(
        const Eigen::Vector3d &p_o_c,    // Posizione Aruco (marker_pos_image)
        const KDL::JntArray &q_curr,
        double K_gain);
private:
    KDLRobot* robot_;
  
    Eigen::Matrix3d toEigen(const KDL::Rotation& r);
    /**
     * @brief Calcola la matrice skew-symmetric S(v) per un vettore 3D.
     */
    inline Eigen::Matrix3d skew(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d s;
        s <<  0, -v(2),  v(1),
             v(2),   0, -v(0),
            -v(1),  v(0),   0;
        return s;
    }

    /**
     * @brief Calcola il task secondario q0_dot per l'evitamento dei limiti.
     */
    Eigen::VectorXd calculate_q0_dot(double lambda,
                                     const KDL::JntArray& q_min,
                                     const KDL::JntArray& q_max);
};

#endif // KDL_CONTROL_H
