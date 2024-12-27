#include "controller/dhc/topt/test_controller.hpp"
#include "controller/dhc/topt/topt_planner.hpp"
#include "controller/my_robot_system/robot_system.hpp"

// Test Controller
 
TestController::TestController(RobotSystem* _robot, 
                              Planner* _planner, 
                              TrajectoryManager* _topt_traj)
  : Controller(_robot, _planner) { 
    rossy_utils::pretty_constructor(1, "Test Controller");
    topt_traj_=_topt_traj;  

    ndof_ = robot_->getNumDofs();
    q_lower_ = robot_->GetPositionLowerLimits();
    q_upper_ = robot_->GetPositionUpperLimits();
    qdot_lower_  = robot_->GetVelocityLowerLimits();
    qdot_upper_ = robot_->GetVelocityUpperLimits();
}

TestController::~TestController(){}

// Get Command through Test
void TestController::getCommand(RobotCommand* cmd){
  static bool b_first = true;
  if(b_first){
    q_cmd_last_ = robot_->getQ();
    b_first = false;
  }   

  Eigen::VectorXd q_cmd, qdot_cmd, qddot_cmd;
  bool bplan = planner_->getPlannedCommand(q_cmd, qdot_cmd, qddot_cmd);
  if(bplan){
    q_cmd_last_ = q_cmd;
    enforcePositionLimits(q_cmd, qdot_cmd);
    enforceVelocityLimits(q_cmd, qdot_cmd);
    cmd->q=q_cmd;
    cmd->qdot=qdot_cmd;
    cmd->qddot=qddot_cmd;
  }else {
    cmd->q=q_cmd_last_;  
    cmd->qdot=Eigen::VectorXd::Zero(q_cmd_last_.size());  
    cmd->qddot=Eigen::VectorXd::Zero(q_cmd_last_.size());  
  }
}

void TestController::enforcePositionLimits(Eigen::VectorXd& q_cmd, 
                                          Eigen::VectorXd& qdot_cmd){
    // check limits
    for(int i(0); i<ndof_; ++i){
      q_cmd(i) = q_cmd(i) > q_lower_(i) ? q_cmd(i) : q_lower_(i);
      q_cmd(i) = q_cmd(i) < q_upper_(i) ? q_cmd(i) : q_upper_(i);
    }
}

void TestController::enforceVelocityLimits(Eigen::VectorXd& q_cmd, 
                                          Eigen::VectorXd& qdot_cmd){
    // check limits
    for(int i(0); i<ndof_; ++i){
      qdot_cmd(i) = qdot_cmd(i) > qdot_lower_(i) ? qdot_cmd(i) : qdot_lower_(i);
      qdot_cmd(i) = qdot_cmd(i) < qdot_upper_(i) ? qdot_cmd(i) : qdot_upper_(i);
    }
}

