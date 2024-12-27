#pragma once


#include <Eigen/Dense>
#include "rossy_utils/io/io_utilities.hpp"

class RobotSystem;

class SensorData {
   public:
    SensorData(int n_qv) {
        elapsedtime = 0.;
        q = Eigen::VectorXd::Zero(n_qv);
        qdot = Eigen::VectorXd::Zero(n_qv);
    }
    virtual ~SensorData() {}

    double elapsedtime;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
};

class RobotCommand {
   public:
    RobotCommand(int n_qv) {
        q = Eigen::VectorXd::Zero(n_qv);
        qdot = Eigen::VectorXd::Zero(n_qv);
        qddot = Eigen::VectorXd::Zero(n_qv);
        jtrq = Eigen::VectorXd::Zero(n_qv);
    }
    virtual ~RobotCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd qddot;
    Eigen::VectorXd jtrq;
};

class EnvInterface{
  protected:
    RobotSystem* robot_;

  public:
    EnvInterface() {}
    virtual ~EnvInterface(){}

    // Get Command through Test
    virtual void updateState(SensorData* _sensor_data) = 0;
    virtual void getCommand(SensorData* _sensor_data, RobotCommand* _command_data) = 0;
    virtual bool doPlanning(void* user_cmd) = 0;
    
};
