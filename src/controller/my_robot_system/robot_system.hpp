#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <string>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>


class RobotSystem {
   protected:
    // pinocchio system
    pinocchio::Model model_;
    pinocchio::Data data_;

    std::string urdf_file_;
    bool b_print_info_;

    // generalized coordinate configuration
    Eigen::VectorXd q_;
    Eigen::VectorXd qdot_;
    Eigen::VectorXd qddot_;

    // robot info
    double total_mass_;
    int n_q_;
    int n_qdot_;
    int n_dof_;
    int n_link_;
    std::vector<int> idx_adof_;
    std::map<std::string, size_t> link_idx_map_;
    std::map<std::string, size_t> joint_idx_map_;
    std::map<size_t, std::string> link_idx_map_inv_;
    std::map<size_t, std::string> joint_idx_map_inv_; 
    
   

   public:
    RobotSystem(const RobotSystem& robotsys); // copier
    RobotSystem(const std::string& file);
    virtual ~RobotSystem(void);

    void printRobotInfo();    
    std::string getUrdfFile(){return urdf_file_;}

    // update fixed base system
    void updateSystem(const Eigen::VectorXd &joint_pos,
                    const Eigen::VectorXd &joint_vel);

    Eigen::VectorXd getQ() { return q_; };
    Eigen::VectorXd getQdot() { return qdot_; };
    Eigen::VectorXd getQddot() { return qddot_; };

    Eigen::VectorXd GetTorqueLowerLimits() { return -model_.effortLimit;}
    Eigen::VectorXd GetTorqueUpperLimits() { return model_.effortLimit;}
    Eigen::VectorXd GetPositionLowerLimits() { return model_.lowerPositionLimit;}
    Eigen::VectorXd GetPositionUpperLimits() { return model_.upperPositionLimit;} 
    Eigen::VectorXd GetVelocityLowerLimits() { return -model_.velocityLimit;}
    Eigen::VectorXd GetVelocityUpperLimits() { return model_.velocityLimit;}   

    
    double getRobotMass() { return total_mass_; }
    int getNumDofs() { return n_qdot_; };
    int getNumBodyNodes() { return n_link_; };

    int getJointIdx(const std::string& joint_name);
    int getLinkIdx(const std::string& frame_name);
    std::string getLinkName(const int& frame_idx);
    std::string getJointName(const int& joint_idx); 

    Eigen::MatrixXd getMassMatrix();
    Eigen::MatrixXd getInvMassMatrix();
    Eigen::VectorXd getCoriolisGravity();
    Eigen::VectorXd getGravity();
    Eigen::VectorXd getCoriolis();
    Eigen::MatrixXd getCoriolisMatrix();

    Eigen::Isometry3d getBodyNodeIsometry(const std::string& name_);
    Eigen::Matrix<double, 6, 1> getBodyNodeSpatialVelocity(const std::string& name_);    
    Eigen::MatrixXd getBodyNodeJacobian(const std::string& name_);
    Eigen::VectorXd getBodyNodeJacobianDotQDot(const std::string& name_);
    Eigen::Matrix<double, 6, 1> getBodyNodeBodyVelocity(const std::string& name_);
    Eigen::MatrixXd getBodyNodeBodyJacobian(const std::string& name_);
    Eigen::VectorXd getBodyNodeBodyJacobianDotQDot(const std::string& name_);

    Eigen::Isometry3d getBodyNodeIsometry(const int& _bn_idx);
    Eigen::Matrix<double, 6, 1> getBodyNodeSpatialVelocity(const int& _bn_idx);    
    Eigen::MatrixXd getBodyNodeJacobian(const int& _bn_idx);
    Eigen::VectorXd getBodyNodeJacobianDotQDot(const int& _bn_idx);
    Eigen::Matrix<double, 6, 1> getBodyNodeBodyVelocity(const int& _bn_idx);
    Eigen::MatrixXd getBodyNodeBodyJacobian(const int& _bn_idx);
    Eigen::VectorXd getBodyNodeBodyJacobianDotQDot(const int& _bn_idx);


  private:
    void _initializeRobotInfo();
    void _updateSystemData();
};
