
#include "stdlib.h"
#include <controller/dhc/topt/test_interface.hpp>
#include <controller/dhc/user_command.hpp>

TestInterface* interface_ = new TestInterface();
SensorData* sensor_data_ = new SensorData(6);
RobotCommand* command_ = new RobotCommand(6);
PlanningCommand* user_command_ = new PlanningCommand();

double randd() {
  return (double)std::rand() / (RAND_MAX + 1.0);
}

Eigen::VectorXd noiseVector(int dim){
    Eigen::VectorXd noise = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim;++i)
        noise[i] = randd() - 0.5;
    return noise;
}

void update(){
    // assume position control
    sensor_data_->elapsedtime += 0.001;
    sensor_data_->q = command_->q + 0.001*noiseVector(6);
    sensor_data_->qdot = command_->qdot + 0.001*noiseVector(6);  
}

int main(int argc, char** argv){

    MOTION_DATA md;
    Eigen::VectorXd q0 = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd p1 = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd p2 = Eigen::VectorXd::Zero(7);
    q0 << 1.57, 0., -1.57, 0., 0., 1.57;
    p1 << 0.6, 0.0, 1.3399999991059304, -0.706433772212892, -0.030843564597231896, 0.030843564597231896, 0.7064337722128922;
    p2 << 0.7257558485970053, -0.19755735983, 1.3357211998562524, -0.706514928645468, -0.028924999586675083, 0.028924999586675083, 0.7065149286454682;
    user_command_->addWayPoint(
        MOTION_DATA(MOTION_DATA_TYPE::JOINT,q0, PATH_TYPE::FREE));
    user_command_->addWayPoint(
        MOTION_DATA(MOTION_DATA_TYPE::POSE, p1, PATH_TYPE::FREE));
    user_command_->addWayPoint(
        MOTION_DATA(MOTION_DATA_TYPE::POSE, p2, PATH_TYPE::FREE));

    int count = 0;
    while(count++ < 10001){

        // if(count == 1000){
        //     interface_ -> doPlanning(user_command_);
        // }            

        // interface_->getCommand(sensor_data_, command_);
        // update();
    }    

    // pinocchio::gepetto::Viewer viewer (model,
    // &visualModel, // or NULL
    // &collisionModel // or NULL
    // );
    // viewer.initViewer("pinocchio"); // window name
    // viewer.loadViewerModel("ur5"); // scene name

    // viewer.display(pin::neutral(model));


    return 0;
}