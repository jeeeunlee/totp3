
#include <math.h>
#include <stdio.h>
#include "controller/dhc/topt/test_interface.hpp"

#include "my_robot_system/robot_system.hpp"
#include "controller/dhc/topt/test_controller.hpp"
#include "controller/dhc/topt/topt_planner.hpp"
#include "controller/dhc/topt/robot_manager.hpp"
#include "controller/dhc/topt/trajectory_manager.hpp"

#include "rossy_utils/general/clock.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "rossy_utils/math/liegroup_utilities.hpp"
#include "rossy_utils/Configuration.h"

TestInterface::TestInterface(int robot_type)
    :EnvInterface(), robot_type_(robot_type) {    
    rossy_utils::color_print(myColor::BoldCyan, rossy_utils::border);
    rossy_utils::pretty_constructor(0, "Test Interface");

    // robot_urdf, link_idx_
    setConfiguration(THIS_COM "config/test.yaml");
        
    // class constructors    
    robot_ = new RobotSystem(robot_urdf_path_);    
    topt_traj_ = new TrajectoryManager();
    planner_ = new ToptPlanner(robot_, robot_type_, link_idx_, topt_traj_);
    controller_ = new TestController( robot_, planner_, topt_traj_);
    clock_ = new Clock();    

    running_time_ = 0.;

    cmd_jpos_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    cmd_jvel_ = Eigen::VectorXd::Zero(robot_->getNumDofs());
    cmd_jtrq_ = Eigen::VectorXd::Zero(robot_->getNumDofs());

    rossy_utils::color_print(myColor::BoldCyan, rossy_utils::border);
}

TestInterface::~TestInterface() {
    delete robot_;
    delete planner_;
    delete controller_;
    delete clock_;
}

void TestInterface::setConfiguration(const std::string& cfgfile){
    try {
        YAML::Node cfg = YAML::LoadFile(cfgfile);
        std::string robot_type = ROBOT_TYPE::names[robot_type_];
        // std::cout<<" robot_type = "<< robot_type.c_str() << std::endl;
        robot_urdf_path_ = THIS_COM;
        robot_urdf_path_ += rossy_utils::readParameter<std::string>(
                            cfg[robot_type], "robot_urdf");
        
        rossy_utils::readParameter(cfg[robot_type], "tool_link_idx", link_idx_);
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}


void TestInterface::getCommand(SensorData* _sensor_data, RobotCommand* _command_data) {
    updateRobotSystem(_sensor_data);   
    controller_->getCommand(_command_data);
    
    // save
    // saveData(_sensor_data, _command_data);
}

void TestInterface::updateState(SensorData* _sensor_data){
    updateRobotSystem(_sensor_data);
}

void TestInterface::updateRobotSystem(SensorData* data){
    robot_->updateSystem(data->q, data->qdot);
    running_time_ = data->elapsedtime;
    ((ToptPlanner*)planner_)->updateTime(running_time_);
}


bool TestInterface::doPlanning(void* user_cmd){
    std::cout<<"DHC: doPlanning start"<<std::endl;    
    clock_->start();
    plan_cmd_ = (PLANNING_COMMAND*)user_cmd;
    bool planned = planner_->doPlanning(plan_cmd_);
    std::cout<<"DHC: doPlanning="<<clock_->stop()<<"ms"<<std::endl;
    return planned;
}

void TestInterface::rePlanning(){
    // running_time_ = data->elapsedtime;
    // ((ToptPlanner*)planner_)->updateTime(running_time_);
    std::cout<<"DHC: rePlanning start"<<std::endl;  
    clock_->start();  
    bool planned = ((ToptPlanner*)planner_)->rePlanning();
    std::cout<<"DHC: rePlanning="<<clock_->stop()<<"ms"<<std::endl;
}

void TestInterface::updateGripData(const GRIP_DATA &gripdata){ 
    ((ToptPlanner*)planner_)->setGripData(gripdata); 
}

void TestInterface::updateVelLimit(const Eigen::VectorXd &vm){
    ((ToptPlanner*)planner_)->setVelLimit(vm);
}

void TestInterface::updateAccLimit(const Eigen::VectorXd &am){
    ((ToptPlanner*)planner_)->setAccLimit(am);
}

void TestInterface::updateJerkLimit(const Eigen::VectorXd &jm){
    ((ToptPlanner*)planner_)->setJerkLimit(jm);
}

void TestInterface::getIKSolution(const Eigen::VectorXd &q0,
             const std::vector<Eigen::VectorXd> &wpts,
             WPT_DATA* qwpts){
    ((ToptPlanner*)planner_)->solveIK(q0, wpts, qwpts->data);  
}

void TestInterface::getFKSolution(const Eigen::VectorXd &q,
                            VEC_DATA* p){
    ((ToptPlanner*)planner_)->solveFK(q, p);
}

void TestInterface::getSysData(WPT_DATA* qwpts, SYSTEM_DATA* sysdata){
    // assume waypoints are given in the joint space
    plan_cmd_->joint_path = qwpts->data;
    ((ToptPlanner*)planner_)->updateSysData(plan_cmd_);
    ((ToptPlanner*)planner_)->getSysData(sysdata);
}


void TestInterface::getTorqueOnTrajectory(TRAJ_DATA* traj_data,
                                        WPT_DATA* torques){
    ((ToptPlanner*)planner_)->computeTorques(traj_data, torques->data);  
}

void TestInterface::getMaxLoadOnTrajectory(const GRIP_DATA& grip_data,
                                            TRAJ_DATA* traj_data){
    
    ((ToptPlanner*)planner_)->computeMaxLoad(grip_data, traj_data);
    std::cout <<" getMaxLoadOnTrajectory " << std::endl;
}

void TestInterface::getSucFrcDistOnTrajectory(const GRIP_DATA& grip_data,
                                            TRAJ_DATA* traj_data,
                                            WPT_DATA* forces){
    std::cout << "DHC: getSucFrcDistOnTrajectory " << std::endl;;
    ((ToptPlanner*)planner_)->computeSuctionForces(
        grip_data, traj_data, forces->data);  
}

void TestInterface::getPlannedResult(const double& time_step,
                                        TRAJ_DATA* traj_data){ 
    std::cout << "DHC: getPlannedResult " << std::endl;;
    // generate desired q, qdot
    Eigen::VectorXd q, qdot;
    Eigen::VectorXd x, xdot;
    double t(0.), tend=topt_traj_->getMotionPeriod();

    // initialize containers
    traj_data->tdata.clear();
    traj_data->qdata.clear();
    traj_data->dqdata.clear();
    traj_data->xdata.clear();
    traj_data->dxdata.clear();
    traj_data->fsucdata.clear();
    traj_data->period = tend;
    traj_data->singularityinpath = ((ToptPlanner*)planner_)->getSingularityCheckResult();

    t = time_step/10.;
    while(t < tend + time_step){
        topt_traj_->getCommand(t, q, qdot);
        t += time_step;
        traj_data->tdata.push_back(t);
        traj_data->qdata.push_back(q);
        traj_data->dqdata.push_back(qdot);

        ((ToptPlanner*)planner_)->solveFK(q, qdot, x, xdot);
        traj_data->xdata.push_back(x.head(3));
        traj_data->dxdata.push_back(xdot.head(3));
        // ((ToptPlanner*)planner_)->computeForceAtSuctionCup(q, qdot,fsuc)
        // traj_data->fsucdata.push_back(fsuc);

        // rossy_utils::saveVector(q, "dhc_data/planned_q");
        // rossy_utils::saveVector(qdot, "dhc_data/planned_qdot");
    }

    // Assume, planned result will be reset
    planner_->reset(); // bplanned = true
}

// SensorData : q, qdot
// RobotCommand : q, qdot, qddot, jtrq
void TestInterface::saveData(SensorData* _sensor_data, RobotCommand* _command_data){    

    rossy_utils::saveValue(running_time_, "test_t");

    // current joint position & joint velocity & acc
    // (1) from robot system
    // Eigen::VectorXd cmd_jvel_prev = cmd_jvel_;    
    // cmd_jpos_ = robot_->getQ();
    // cmd_jvel_ = robot_->getQdot();
    // cmd_jacc_ = (cmd_jvel_- cmd_jvel_prev)/0.001;

    // (2) from SensorData
    // Eigen::VectorXd cmd_jvel_prev = cmd_jvel_;    
    // cmd_jpos_ = _sensor_data->q;
    // cmd_jvel_ = _sensor_data->qdot;
    // cmd_jacc_ = (cmd_jvel_- cmd_jvel_prev)/0.001;

    // (3) from RobotCommand
    Eigen::VectorXd cmd_jvel_prev = cmd_jvel_;    
    cmd_jpos_ = _command_data->q;
    cmd_jvel_ = _command_data->qdot;
    cmd_jacc_ = _command_data->qddot;
   
    // current joint trq
    Eigen::MatrixXd M = robot_->getMassMatrix();
    Eigen::VectorXd cori = robot_->getCoriolisGravity();
    cmd_jtrq_ = M*cmd_jacc_ + cori;    

    rossy_utils::saveVector(cmd_jpos_, "test_q");
    rossy_utils::saveVector(cmd_jvel_, "test_qdot");
    rossy_utils::saveVector(cmd_jacc_, "test_qddot");
    rossy_utils::saveVector(cmd_jtrq_, "test_trq");    

    // current EE position
    Eigen::VectorXd pose;
    rossy_utils::convertIsoToVec7d(
        robot_->getBodyNodeIsometry(link_idx_), pose);
    rossy_utils::saveVector(pose, "test_EE");
    
}

// generate data for dex gripper with 8 suction cups
DEX_GRIPPER_V4::DEX_GRIPPER_V4(){
    // suction cup info
    double r_pad = 0.063/2.;
    double mu = 0.7; // 0.7

    // gripper geometry info
    p_tc << 0.0825, 0., 0.; // tool to grpr ctr
    double d = 0.083; // distance b/w cups
    double f_suction = 78;  // 78 * 0.9; // [N]
    setGripper(d, r_pad, mu, f_suction);
    
    double mass = 0.5; // 2 [kg]
    double rcom = 0.2; // 0.2 [m] box length
    double pcom = rcom; // tool to object
    setObject(mass, rcom, pcom);
}
  
// BUILD fs, G, U
void DEX_GRIPPER_V4::setGripper(
        double d, double r_pad, double mu, double f){
    // x0, z0 : the position of {0} from tool frame
    // d : distance btw suction cups
    // h : height (=y0)
    // rpad : radius of suctio cup

    // [Tool frame]         +z
    //      {7}   {5}   {3}   {1}
    // +x            ctr    +
    //      {6}   {4}   {2}   {0}
    // [Suction cup frame]     +x
    //                     +y (+z)
    // # of suction cups
    n = 8;
    rpads = Eigen::VectorXd::Constant(n, r_pad);
    mus = Eigen::VectorXd::Constant(n, mu);  
    // force in suction frame  
    Eigen::VectorXd f6d {{0.,0.,0., 0.,0.,-f}}; 
    // rotation matrix from tool to suction cup
    Eigen::MatrixXd R {{0,1,0}, {0,0,1}, {1,0,0}};    
    // suction cup position from tool frame
    std::vector<Eigen::VectorXd> plist;
    Eigen::VectorXd p = p_tc;
    for(int i(0); i <4; ++i){
        for(int j(0); j<2; ++j){
            p<< p_tc(0) + d*((double)i-1.5),
                p_tc(1),
                p_tc(2) + d*((double)j-0.5);
            plist.push_back(p);
        }
    }

    //---- Compute fs, G : configuration matrix
    Eigen::MatrixXd Tit, adT;
    G = Eigen::MatrixXd::Zero(0,0);
    fs = Eigen::VectorXd::Zero(0); 
    for(int i(0); i<n; ++i){
        p = plist[i];      
        Tit = rossy_utils::InverseSE3(R,p);
        adT = rossy_utils::Ad_T(Tit);      
        G = rossy_utils::hStack(G, adT.transpose());
        fs = rossy_utils::vStack(fs, f6d);
    }

    //---- compute U : inequality matrix 
    p = Eigen::VectorXd::Zero(3); 
    Eigen::MatrixXd Tti = rossy_utils::SE3(R,p);
    Eigen::MatrixXd Ad_Tti = rossy_utils::Ad_T(Tti);   
    double xmax = 0.5*d;
    double ymax = 1.5*d +  p_tc(0);  
    _computeFrictionConeMatrix(n, mu, r_pad, xmax, ymax, Ad_Tti);
}

// generate data for dex gripper with 7 suction cups
DEX_GRIPPER_V3::DEX_GRIPPER_V3():GRIP_DATA(){
    // suction cup info
    double r_pad = 0.063/2.;
    double mu = 0.7; // 0.7
    // gripper geometry info
    p_tc << 0., 0., 0.; // tool to grpr ctr
    double r = 0.08; // 0.1
    double f_suction = 78; // 78; // [N]
    setGripper(r, r_pad, mu, f_suction);
    
    double mass = 0.5; // 2 [kg]
    double rcom = 0.2; //0.2 [m] box length
    double pcom = rcom; // tool to object
    setObject(mass, rcom, pcom);
}
  
// BUILD fs, G, U
void DEX_GRIPPER_V3::setGripper(
        double r, double r_pad, double mu, double f){
    // # of suction cups
    n = 7;
    rpads = Eigen::VectorXd::Constant(n, r_pad);
    mus = Eigen::VectorXd::Constant(n, mu);
    // force in suction cup frame
    Eigen::VectorXd f6d {{0.,0.,0., 0.,0.,-f}}; 
    // [tool frame]..............[Suction cup frame]
    //          +z ..............
    //      {2}   {3} ............     +x
    // +x {7}  {1}  {4} .......... +y (+z)
    //      {6}   {5} ............

    // rotation matrix from tool to suction cup
    Eigen::MatrixXd R {{0,1,0}, {0,0,1}, {1,0,0}};

    std::vector<Eigen::VectorXd> plist;
    // 0:center
    Eigen::VectorXd p = p_tc;
    plist.push_back(p);
    // 1~6
    for(int i(1); i<n; ++i){
        double theta = (double)i * M_PI/3.;
        // std::cout<<"theta = "<<theta << ", ";
        p<< p_tc(0) + r*cos(theta),
            p_tc(1),
            p_tc(2) + r*sin(theta);
        plist.push_back(p);
    }

    //---- Compute fs, G : configuration matrix
    Eigen::MatrixXd Tit, adT;
    G = Eigen::MatrixXd::Zero(0,0);
    fs = Eigen::VectorXd::Zero(0); 
    for(int i(0); i<n; ++i){
        p = plist[i];      
        Tit = rossy_utils::InverseSE3(R,p);
        adT = rossy_utils::Ad_T(Tit);      
        G = rossy_utils::hStack(G, adT.transpose());
        fs = rossy_utils::vStack(fs, f6d);
    }

    // compute U
    p = Eigen::VectorXd::Zero(3); 
    Eigen::MatrixXd Tti = rossy_utils::SE3(R,p);
    Eigen::MatrixXd Ad_Tti = rossy_utils::Ad_T(Tti);  
    double xmax = sqrt(3.)/2.*r;
    double ymax = r;
    _computeFrictionConeMatrix(n, mu, r_pad, xmax, ymax, Ad_Tti);
    
}



