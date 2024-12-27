#pragma once

#include "controller/dhc/interface.hpp"
#include "controller/dhc/user_command.hpp"



class Planner;
class Controller;
class Clock;
class TrajectoryManager;
class RobotManager;

// to generate data for dex gripper with 7 suction cups
class DEX_GRIPPER_V3: public GRIP_DATA {
   public:
      DEX_GRIPPER_V3();
      ~DEX_GRIPPER_V3(){};

   private:
      void setGripper(double r, double r_pad, double mu, double f);

};
// to generate data for dex gripper with 8 suction cups
class DEX_GRIPPER_V4: public GRIP_DATA {
   public:
      DEX_GRIPPER_V4();
      ~DEX_GRIPPER_V4(){};
   private:
      void setGripper(double d, double r_pad, double mu, double f);
};

class TestInterface : public EnvInterface {
   protected:
      double running_time_;

      int link_idx_;      
      std::string robot_urdf_path_;
      int robot_type_; // :rs020n

      RobotCommand* cmd_;
      SensorData* data_;
      PLANNING_COMMAND* plan_cmd_;

      Planner* planner_;
      Controller* controller_;
      TrajectoryManager* topt_traj_;      
      
      Clock* clock_;     

      Eigen::VectorXd cmd_jpos_;
      Eigen::VectorXd cmd_jvel_;
      Eigen::VectorXd cmd_jacc_;
      Eigen::VectorXd cmd_jtrq_;

   public:
      TestInterface(int robot_type);
      ~TestInterface();
      
      virtual void getCommand(SensorData* _sensor_data, RobotCommand* _command_data);
      virtual bool doPlanning(void* user_cmd);
      virtual void updateState(SensorData* _sensor_data);

      void rePlanning();
      void updateGripData(const GRIP_DATA &gripdata);
      void updateVelLimit(const Eigen::VectorXd &vm);
      void updateAccLimit(const Eigen::VectorXd &am);
      void updateJerkLimit(const Eigen::VectorXd &jm);
           
      
      void getIKSolution(const Eigen::VectorXd &q0,
                        const std::vector<Eigen::VectorXd> &wpts,             
                        WPT_DATA* qwpts);
      void getFKSolution(const Eigen::VectorXd &q0,
                           VEC_DATA* p);
      void getSysData(WPT_DATA* qwpts, SYSTEM_DATA* sysdata);
      void getPlannedResult(const double& time_step,
                           TRAJ_DATA* traj_data);
      void getTorqueOnTrajectory(TRAJ_DATA* traj_data,
                                 WPT_DATA* torques);
      void getMaxLoadOnTrajectory(const GRIP_DATA& grip_data, 
                                 TRAJ_DATA* traj_data);
      void getSucFrcDistOnTrajectory(const GRIP_DATA& grip_data,
                                    TRAJ_DATA* traj_data,
                                    WPT_DATA* forces);

   private:
      void updateRobotSystem(SensorData * data); 
      void saveData(SensorData* _sensor_data, RobotCommand* _command_data);
      void setConfiguration(const std::string& cfgfile);    

};
