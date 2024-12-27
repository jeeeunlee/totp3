#pragma once

#include <deque>

#include "controller/dhc/planner.hpp"
#include "controller/dhc/user_command.hpp"

class ToptSolver;
class TrajectoryManager;
class RobotManager;

namespace REDIST_TYPE{
  constexpr int NONE=0;
  constexpr int UNIFORM=1;
  constexpr int TOPPRA=2;
  constexpr int UNIFORMWEIGHT=3;
  constexpr int UNIFORMNORM=4;
}

// Time Optimal Trajectory Planner
class ToptPlanner: public Planner{
  protected:    
    int link_idx_;
    int n_dof_;
    int robot_type_;     

    // updatable through the interface
    Eigen::VectorXd vel_limit_;
    Eigen::VectorXd acc_limit_;
    Eigen::VectorXd jerk_limit_;

    GRIP_DATA grip_data_;
    SYSTEM_DATA sys_data_;    
    bool is_singularity_in_path_;

    RobotSystem* robot_temp_;    
    ToptSolver* topt_solver_;
    TrajectoryManager* topt_traj_;
    RobotManager* robot_manager_;

    double threshold_pinv_;
  public:
    ToptPlanner(RobotSystem* _robot, int _robot_type,
                int _link_idx, TrajectoryManager* _topt_traj);
    ~ToptPlanner();

    bool doPlanning(PLANNING_COMMAND* planning_cmd);
    bool rePlanning();
    bool getPlannedCommand(Eigen::VectorXd& q_cmd);
    bool getPlannedCommand(Eigen::VectorXd& q_cmd,
                          Eigen::VectorXd& qdot_cmd);
    bool getPlannedCommand(Eigen::VectorXd& q_cmd,
                          Eigen::VectorXd& qdot_cmd,
                          Eigen::VectorXd& qddot_cmd);

    void preProcessingCommand(
        PLANNING_COMMAND* planing_cmd,
        int redist_mode = REDIST_TYPE::UNIFORM);    
    void checkSingularity(std::vector<Eigen::VectorXd>& qwpts);
    bool getSingularityCheckResult();
    void updateSysData(PLANNING_COMMAND* planning_cmd);
    void getSysData(SYSTEM_DATA* sysdata){*(sysdata)=sys_data_;}

    // data setting for interface
    void setGripData(const GRIP_DATA &gripdata); 
    void setVelLimit(const Eigen::VectorXd &vm);   
    void setAccLimit(const Eigen::VectorXd &am);
    void setJerkLimit(const Eigen::VectorXd &jm);

    void solveIK(const Eigen::VectorXd &q0,
             const std::vector<Eigen::VectorXd> &wpts,
             std::vector<Eigen::VectorXd> &qwpts);

    void solveFK(const Eigen::VectorXd &q,
                  VEC_DATA* p);
    void solveFK(const Eigen::VectorXd &q,
                const Eigen::VectorXd &qdot,
                Eigen::VectorXd &x,
                Eigen::VectorXd &xdot);
    void computeTorques(TRAJ_DATA *traj_data,
             std::vector<Eigen::VectorXd> &qwpts);
    void computeMaxLoad(const GRIP_DATA &gripdata,
                        TRAJ_DATA *traj_data);
    void computeUnitLoad(const GRIP_DATA &gripdata,
                        TRAJ_DATA *traj_data);
    void computeSuctionForces(const GRIP_DATA &gripdata,
            TRAJ_DATA *traj_data,                
            std::vector<Eigen::VectorXd> &frc_data);

};

