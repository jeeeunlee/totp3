#pragma once

#include "controller/dhc/interface.hpp"

class Planner;

class Controller{  
  protected:
    RobotSystem* robot_;
    Planner* planner_;

  public:   
    Controller(RobotSystem* _robot, Planner* _planner){
        robot_ = _robot;
        planner_ =_planner; }

    ~Controller(){};

    // Get Command through Test
    virtual void getCommand(RobotCommand* _cmd) = 0;
    // planner_
    // ((RobotCommand*)_cmd)->jtrq = jtrq_des_;
    // ((RobotCommand*)_cmd)->q = jpos_des_;
    // ((RobotCommand*)_cmd)->qdot = jvel_des_;
    
    

};