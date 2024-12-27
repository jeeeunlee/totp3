#pragma once

#include <controller/dhc/controller.hpp>

class Planner;
class ToptPlanner;
class TrajectoryManager;

// Test Controller
class TestController: public Controller{  
  public:   
    TestController(RobotSystem* _robot, 
                  Planner* _planner,
                  TrajectoryManager* _topt_traj);
    ~TestController();

    // Get Command through Test
    void getCommand(RobotCommand* _cmd);

    Eigen::VectorXd q_cmd_last_;
    TrajectoryManager* topt_traj_;

  private:
    void enforcePositionLimits(Eigen::VectorXd& q_cmd, 
                              Eigen::VectorXd& qdot_cmd);
    void enforceVelocityLimits(Eigen::VectorXd& q_cmd, 
                              Eigen::VectorXd& qdot_cmd);
    int ndof_;
    Eigen::VectorXd q_lower_;
    Eigen::VectorXd q_upper_;
    Eigen::VectorXd qdot_lower_;
    Eigen::VectorXd qdot_upper_;


};