#pragma once

#include <deque>
#include <Eigen/Dense>

class RobotSystem;
class PLANNING_COMMAND;

class Planner{
  protected:
    RobotSystem* robot_;
    double current_time_ =0.;
    double start_time_=0.;
    double planned_time_=0.;
    double end_time_=0.;

    bool b_planned_=false;
    bool b_planned_firstvisit_=false;

  public:
    Planner(RobotSystem* _robot){
        robot_ = _robot; }
    virtual ~Planner(){}

    virtual bool doPlanning(PLANNING_COMMAND* _user_cmd) = 0;
    virtual bool getPlannedCommand(Eigen::VectorXd& q_cmd) = 0;
    virtual bool getPlannedCommand(Eigen::VectorXd& q_cmd,
                                Eigen::VectorXd& qdot_cmd) = 0;
    virtual bool getPlannedCommand(Eigen::VectorXd& q_cmd,
                                Eigen::VectorXd& qdot_cmd,
                                Eigen::VectorXd& qddot_cmd) = 0;

    void reset(){b_planned_=false;}
    void updateTime(double _t){
      current_time_ = _t;
      if(b_planned_ && 
          (current_time_ > end_time_)){
              b_planned_ = false;
              // std::cout<<" start_time_ = "<<start_time_<<std::endl;
              // std::cout<<" current_time_ = "<<current_time_<<std::endl;
              // std::cout<<" end_time_ = "<<end_time_<<std::endl;
              // std::cout<<"b_planned changed!!"<<std::endl;
          }
    }
};