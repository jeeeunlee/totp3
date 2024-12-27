#pragma once

#include <Eigen/Dense>
class RobotSystem;

namespace ROBOT_TYPE{
  constexpr int RS020N=0;

  const std::array<std::string, 1> names = {
     "rs020n"
  };
}

class RobotManager{
  protected:
    int link_idx_;
    int n_dof_;
    double threshold_pinv_;    
    RobotSystem* robot_temp_;
  public:
    RobotManager(RobotSystem* robot_temp, 
                int link_idx, 
                double threshold_pinv);
    ~RobotManager(){};
    virtual void solveIK(const Eigen::VectorXd &q0,
              const std::vector<Eigen::VectorXd> &wpts,
              std::vector<Eigen::VectorXd> &qwpts);

    
};

class RS020NManager: public RobotManager{
  public: 
    RS020NManager(RobotSystem* robot_temp, 
                  int link_idx, 
                  double thre_pinv); 
};
