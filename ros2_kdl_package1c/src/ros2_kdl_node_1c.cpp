// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include "rclcpp/qos.hpp"

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"

#include "kdl_robot.h"
#include "kdl_control.h" // Assicurati che kdl_control.h sia nel percorso
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class Iiwa_pub_sub : public rclcpp::Node
{
    public:
        Iiwa_pub_sub()
        : Node("ros2_kdl_node_1b"),
         node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
        {
            // ... (dichiarazioni parametri esistenti: cmd_interface, traj_type, s_type) ...
            
            // declare cmd_interface parameter
            declare_parameter("cmd_interface", "position"); 
            get_parameter("cmd_interface", cmd_interface_);
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
            if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort" ))
            {
                RCLCPP_ERROR(get_logger(),"Selected cmd interface is not valid! Use 'position', 'velocity' or 'effort' instead..."); return;
            }

            // ... (dichiarazioni parametri traj_type, s_type) ...
            declare_parameter("traj_type", "linear");
            get_parameter("traj_type", traj_type_);
            // ... (check) ...

            declare_parameter("s_type", "trapezoidal");
            get_parameter("s_type", s_type_);
            // ... (check) ...


            // declare ctrl parameter (AGGIUNTO 'vision')
            declare_parameter("ctrl", "velocity_ctrl"); 
            get_parameter("ctrl", ctrl_);
            RCLCPP_INFO(get_logger(),"Current velocity controller is: '%s'", ctrl_.c_str());
            // *** AGGIUNTO 'vision' AL CHECK *** [cite: 36]
            if (!(ctrl_ == "velocity_ctrl" || ctrl_ == "velocity_ctrl_null" || ctrl_ == "vision"))
            {
                RCLCPP_ERROR(get_logger(),"Selected ctrl is not valid! Use 'velocity_ctrl', 'velocity_ctrl_null' or 'vision' instead..."); return;
            }



            // ... (dichiarazioni parametri esistenti: traj_duration, ecc.) ...
            declare_parameter("traj_duration", 1.5);
            get_parameter("traj_duration", traj_duration_);
            declare_parameter("acc_duration", 0.5);
            get_parameter("acc_duration", acc_duration_);
            declare_parameter("total_time", 1.5);
            get_parameter("total_time", total_time_);
            declare_parameter("trajectory_len", 150);
            get_parameter("trajectory_len", trajectory_len_);
            declare_parameter("Kp", 5.0);
            get_parameter("Kp", Kp_);
            declare_parameter("end_position_x", 0.0);
            get_parameter("end_position_x", end_position_x_);
            declare_parameter("end_position_y", 0.0);
            get_parameter("end_position_y", end_position_y_);
            declare_parameter("end_position_z", 0.0);
            get_parameter("end_position_z", end_position_z_);

            // *** NUOVI PARAMETRI PER IL CONTROLLORE (2b) ***
            declare_parameter("Kp_vision", 1.0); // Guadagno K per vision_ctrl [cite: 39]
            get_parameter("Kp_vision", Kp_vision_);
            declare_parameter("lambda_null", 0.1); // Guadagno lambda per spazio nullo [cite: 19]
            get_parameter("lambda_null", lambda_null_);


            iteration_ = 0; t_ = 0;
            joint_state_available_ = false; 
            aruco_pose_available_ = false; // *** Flag per posa Aruco
            p_o_c_.setZero();              // *** Vettore posizione Aruco

            // ... (retrieve robot_description param) ...
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            // ... (wait_for_service) ...
            auto parameter = parameters_client->get_parameters({"robot_description"});

            // ... (create KDLrobot structure) ...
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array (*** MODIFICATO PER USARE MEMBRI DELLA CLASSE q_min_ / q_max_ ***)
            unsigned int nj = robot_->getNrJnts();
            q_min_.resize(nj); // *** Membro della classe
            q_max_.resize(nj); // *** Membro della classe
            q_min_.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; 
            q_max_.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;          
            robot_->setJntLimits(q_min_,q_max_); // *** Usa membri della classe           
            joint_positions_.resize(nj); 
            joint_velocities_.resize(nj); 
            joint_positions_cmd_.resize(nj); 
            joint_velocities_cmd_.resize(nj); 
            joint_efforts_cmd_.resize(nj); joint_efforts_cmd_.data.setZero();

            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

            arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 
            rclcpp::SensorDataQoS(), 
            std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));
            
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }
            
            // Se in modalità vision, attendi anche il marker
            if (ctrl_ == "vision") {
                RCLCPP_INFO(this->get_logger(), "Modalità VISION attiva. In attesa del topic Aruco...");
            }


            // ... (Update KDLrobot object, Compute EE frame, Compute IK) ...
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            init_cart_pose_ = robot_->getEEFrame();
            // ... (IK test) ...

            // Initialize controller
            controller_ = std::make_shared<KDLController>(*robot_); 


            // --- Pianificazione Traiettoria (per modalità non-vision) ---
            if (ctrl_ != "vision")
            {
                Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));
                Eigen::Vector3d end_position; 
                end_position << end_position_x_, end_position_y_, end_position_z_;
                RCLCPP_INFO(get_logger(), "Trajectory End Position set to: [%f, %f, %f]", end_position[0], end_position[1], end_position[2]);

                double traj_radius = 0.15; 
                if(traj_type_ == "linear"){
                    planner_ = KDLPlanner(traj_duration_, acc_duration_, init_position, end_position); 
                    if(s_type_ == "trapezoidal") { p_ = planner_.linear_traj_trapezoidal(t_); }
                    else if(s_type_ == "cubic") { p_ = planner_.linear_traj_cubic(t_); }
                } 
                else if(traj_type_ == "circular") {
                    planner_ = KDLPlanner(traj_duration_, init_position, traj_radius, acc_duration_);
                    if(s_type_ == "trapezoidal") { p_ = planner_.circular_traj_trapezoidal(t_); }
                    else if(s_type_ == "cubic") { p_ = planner_.circular_traj_cubic(t_); }
                }
            }


            long int timer_ms = 100;
            
            if(cmd_interface_ == "position"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }
            else if(cmd_interface_ == "effort"){
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_ms), 
                                            std::bind(&Iiwa_pub_sub::cmd_publisher, this));
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            } 

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);

            if (ctrl_ == "vision") {
                RCLCPP_INFO(this->get_logger(), "Starting Vision-Based Control execution...");
            } else {
                RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
            }
        }

    private:

        void cmd_publisher(){

            iteration_ = iteration_ + 1;

            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

                // Incrementa il tempo solo se NON siamo in modalità vision
                if (ctrl_ != "vision") {
                    double dt = 0.01; // Assumendo timer a 10ms (100Hz)
                    t_ += dt;
                }

    
            if (t_ < total_time_ || ctrl_ == "vision"){
    
                if (ctrl_ != "vision") {
                    if(traj_type_ == "linear"){
                        if(s_type_ == "trapezoidal") { p_ = planner_.linear_traj_trapezoidal(t_); }
                        else if(s_type_ == "cubic") { p_ = planner_.linear_traj_cubic(t_); }
                    } 
                    else if(traj_type_ == "circular") {
                        if(s_type_ == "trapezoidal") { p_ = planner_.circular_traj_trapezoidal(t_); }
                        else if(s_type_ == "cubic") { p_ = planner_.circular_traj_cubic(t_); }
                    }
                }

                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           

 
                Eigen::Vector3d error = Eigen::Vector3d::Zero();
                Eigen::Vector3d o_error = Eigen::Vector3d::Zero();
                if (ctrl_ != "vision") {
                    error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
                    o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                    std::cout << "The error norm is : " << error.norm() << std::endl;
                }

                else if(cmd_interface_ == "velocity"){
                    

                            if (ctrl_ == "velocity_ctrl") {
                            // Controllo traiettoria classica
                            Vector6d cartvel; cartvel << p_.vel + Kp_*error, o_error;
                            joint_velocities_cmd_ = controller_->velocity_ctrl(cartvel); 
                        }
                        else if (ctrl_ == "vision")
                    {
                        if (!aruco_pose_available_)
                        {
                            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "Waiting for ArUco...");
                            joint_velocities_cmd_.data.setZero();
                        }
                        else
                      
                            
                            Eigen::VectorXd q_curr_vec = toEigen(joint_positions_);
                        
                            Eigen::VectorXd cmd_vec = controller_->velocity_ctrl_vision(
                                p_o_c_,      // Posizione Aruco
                                joint_positions_, // Posizione giunti attuale (KDL::JntArray)
                                Kp_vision_   // Guadagno
                            );
                            
            
                            for(int i=0; i<cmd_vec.size(); i++) {
                                joint_velocities_cmd_(i) = cmd_vec(i);
                            }
                        }
                    }
                
                

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

                // ... (Imposta i comandi desiderati in base a cmd_interface_) ...
                if(cmd_interface_ == "position"){
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                } 

                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            else{ // Questo blocco ora viene eseguito solo se t_ >= total_time_ E non siamo in modalità vision
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                
                // ... (logica di stop esistente) ...
                if(cmd_interface_ == "position"){
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_cmd_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = 0.0;
                    }
                }
                else if(cmd_interface_ == "effort"){
                    for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                        desired_commands_[i] = joint_efforts_cmd_(i);
                    }
                }
                
                std_msgs::msg::Float64MultiArray cmd_msg;
                std::fill(desired_commands_.begin(), desired_commands_.end(), 0.0);
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
        }

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
            // ... (codice esistente non modificato) ...
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        // *** NUOVO CALLBACK PER ARUCO POSE (Punto 2b) ***
        void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped& msg)
        {

            p_o_c_(0) = msg.pose.position.x;
            p_o_c_(1) = msg.pose.position.y;
            p_o_c_(2) = msg.pose.position.z;
            
            if (!aruco_pose_available_) {
                RCLCPP_INFO(get_logger(), "Prima posa Aruco ricevuta: [x: %f, y: %f, z: %f]",
                            p_o_c_(0), p_o_c_(1), p_o_c_(2));
            }
            aruco_pose_available_ = true;
        }


        // --- Variabili Membro ---
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
        // *** NUOVO SUBSCRIBER ***
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_; 
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        
        // *** NUOVI MEMBRI PER LIMITI GIUNTI ***
        KDL::JntArray q_min_;
        KDL::JntArray q_max_;

        KDL::JntArray joint_positions_cmd_;
        KDL::JntArray joint_velocities_cmd_;
        KDL::JntArray joint_efforts_cmd_;

        std::shared_ptr<KDLRobot> robot_;
        std::shared_ptr<KDLController> controller_; 
        KDLPlanner planner_;

        trajectory_point p_;

        int iteration_;
        bool joint_state_available_;
        // *** NUOVI MEMBRI PER VISIONE ***
        bool aruco_pose_available_;
        Eigen::Vector3d p_o_c_; // Posizione oggetto in frame camera

        double t_;
        std::string cmd_interface_;
        std::string traj_type_;
        std::string s_type_;
        std::string ctrl_; 

        KDL::Frame init_cart_pose_;

        double traj_duration_;
        double acc_duration_;
        double total_time_;
        int    trajectory_len_;
        double Kp_;
        double end_position_x_;
        double end_position_y_;
        double end_position_z_;
        
        // *** NUOVI MEMBRI PER PARAMETRI CONTROLLO ***
        double Kp_vision_;
        double lambda_null_;
    }; 
 
int main( int argc, char** argv )
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}
