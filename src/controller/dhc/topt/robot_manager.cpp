#include "controller/dhc/topt/robot_manager.hpp"
#include "controller/my_robot_system/robot_system.hpp"
#include "rossy_utils/io/io_utilities.hpp"
#include "rossy_utils/math/pseudo_inverse.hpp"
#include "rossy_utils/math/math_utilities.hpp"

RobotManager::RobotManager(RobotSystem* robot_temp, 
                            int link_idx, 
                            double threshold_pinv){
    robot_temp_ = robot_temp;
    link_idx_ = link_idx;
    threshold_pinv_ = threshold_pinv;
}

void RobotManager::solveIK(const Eigen::VectorXd &q0, 
                            const std::vector<Eigen::VectorXd> &wpts, 
                            std::vector<Eigen::VectorXd> &qwpts)
{
    // std::cout<<"solveIK"<<std::endl;

    qwpts.clear();    
    int n_wpts = wpts.size();
    Eigen::VectorXd q = q0;
    Eigen::Quaternion<double> ori_des, ori_act, quat_ori_err;
    Eigen::VectorXd pdes, delp, delq;
    Eigen::MatrixXd J, invJ;

    // initial
    double dq_max = 0.1*M_PI;
    double alpha=1.;
    while(alpha>1e-3){
        robot_temp_->updateSystem(q, 0.0*q);
        pdes = wpts[0];
        delp = Eigen::VectorXd::Zero(6);
        // position
        delp.tail(3) = pdes.head(3) - 
            robot_temp_->getBodyNodeIsometry(link_idx_).translation();
        // orientation        
        ori_des = Eigen::Quaternion<double>(
            pdes[3], pdes[4], pdes[5], pdes[6]);
        ori_act = Eigen::Quaternion<double>(
            robot_temp_->getBodyNodeIsometry(link_idx_).linear());            
        quat_ori_err = ori_des * (ori_act.inverse());
        delp.head(3) = rossy_utils::convertQuatToExp(quat_ori_err);        
        J = robot_temp_->getBodyNodeJacobian(link_idx_);
        rossy_utils::pseudoInverse(J, threshold_pinv_, invJ);
        delq = invJ*delp;
        alpha = std::max(1., delq.norm()/dq_max);
        q += 1./alpha*delq;
        alpha=delq.norm();
    }

    for(int i(0);i<n_wpts; ++i)
    {
        robot_temp_->updateSystem(q, 0.0*q);
        pdes = wpts[i]; // x,y,z, w,x,y,z
        delp = Eigen::VectorXd::Zero(6);
        // position
        delp.tail(3) = pdes.head(3) - 
            robot_temp_->getBodyNodeIsometry(link_idx_).translation();
        // orientation        
        ori_des = Eigen::Quaternion<double>(
            pdes[3], pdes[4], pdes[5], pdes[6]);
        ori_act = Eigen::Quaternion<double>(
            robot_temp_->getBodyNodeIsometry(link_idx_).linear());            
        quat_ori_err = ori_des * (ori_act.inverse());
        delp.head(3) = rossy_utils::convertQuatToExp(quat_ori_err);        
        J = robot_temp_->getBodyNodeJacobian(link_idx_);
        rossy_utils::pseudoInverse(J, threshold_pinv_, invJ);
        delq = invJ*delp;
        q += delq;

        qwpts.push_back(q);
        // rossy_utils::pretty_print(q,std::cout, "IK:q");
    }
}

RS020NManager::RS020NManager(RobotSystem *robot_temp, 
                            int link_idx, double thre_pinv)
: RobotManager(robot_temp,link_idx,thre_pinv){
    rossy_utils::pretty_constructor(2, "RS020NManager");
}