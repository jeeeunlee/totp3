
#include "stdlib.h"
#include <controller/dhc/topt/test_interface.hpp>
#include <controller/dhc/user_command.hpp>



// #include <pinocchio/gepetto/viewer.hpp>

TestInterface* interface_ = new TestInterface();
SensorData* sensor_data_ = new SensorData(6);
RobotCommand* command_ = new RobotCommand(6);
PlanningCommand* user_command_ = new PlanningCommand();

int main(int argc, char** argv){
                   

    interface_->doPlanning(user_command_);

    return 0;
}