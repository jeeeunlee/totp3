#pragma once


#include <iostream>
#include <Eigen/Dense>
#include <rossy_utils/io/io_utilities.hpp>

class TRAJ_DATA{
  public:
    TRAJ_DATA(){  
      tdata.clear();    
      qdata.clear();
      dqdata.clear();
      xdata.clear();
      dxdata.clear();
      fsucdata.clear();
      period=0.;
      singularityinpath = false;
      }
    ~TRAJ_DATA(){}
  public:
    std::vector< double > tdata;
    std::vector< Eigen::VectorXd > qdata;
    std::vector< Eigen::VectorXd > dqdata;
    std::vector< Eigen::VectorXd > xdata;
    std::vector< Eigen::VectorXd > dxdata;
    std::vector< Eigen::VectorXd > fsucdata;
    double period;
    bool singularityinpath;
};

class PLANNING_COMMAND{
public:
    PLANNING_COMMAND(){      
      joint_path.clear();
      cartesian_path.clear();

      // should be set
      max_joint_acceleration={};
      max_joint_speed={};
      max_joint_jerk={};

      max_linear_acceleration = -1.;
      max_linear_speed = -1.;

      acc_percentage_path_ratio = 0.;
      acc_percentage = 1.;
      dec_percentage_path_ratio = 0.;
      dec_percentage = 1.;
      }
    ~PLANNING_COMMAND(){}
  public:
    std::vector< Eigen::VectorXd > joint_path;
    std::vector< Eigen::VectorXd > cartesian_path;
    
    Eigen::VectorXd max_joint_acceleration;
    Eigen::VectorXd max_joint_speed;
    Eigen::VectorXd max_joint_jerk;

    double max_linear_acceleration;
    double max_linear_speed;

    double acc_percentage_path_ratio;
    double acc_percentage;
    double dec_percentage_path_ratio;
    double dec_percentage;
};

class WPT_DATA{
  public:
    WPT_DATA(){
      b_cartesian = true;
      data.clear();}
    ~WPT_DATA(){}

    int getsize(){return data.size();}
    Eigen::VectorXd getdata(int i){
      if(i>0 && i<data.size()) return data[i];
      else return Eigen::VectorXd::Zero(0);
    }
  public:
    bool b_cartesian;
    std::vector< Eigen::VectorXd > data;
};

class VEC_DATA{
  public:
  Eigen::VectorXd data;
  VEC_DATA(){}
  ~VEC_DATA(){}
};

class GRIP_DATA{
  public:
    GRIP_DATA(){};
    ~GRIP_DATA(){};

    //  object parameters
    //  double m; // mass 
    //  Eigen::MatrixXd I; // Inertia in object frame
    //  Eigen::VectorXd p; // from tool to object frame
    //  Eigen::VectorXd p_tc; // from tool to tool center
    void setObject(double mass, double rcom, double pcom){
      m = mass;
      I1 = 0.5*rcom*rcom*Eigen::MatrixXd::Identity(3,3);
      I = mass*I1;
      p = Eigen::VectorXd::Zero(3);
      p << p_tc(0), p_tc(1)+pcom, p_tc(2);
    }

    void setBoxObject(double mass, double box_x, double box_y, 
                      double box_z, double pcom){
        m = mass;
        // assume ellipsoid, assume x,y,z comes in tool coordinate    
        I1 = Eigen::MatrixXd::Identity(3,3);
        I1(0,0)=0.333*0.5*0.5*(box_y*box_y + box_z*box_z);
        I1(1,1)=0.333*0.5*0.5*(box_x*box_x + box_z*box_z);
        I1(2,2)=0.333*0.5*0.5*(box_y*box_y + box_x*box_x);
        I = mass*I1;        
        p = Eigen::VectorXd::Zero(3);
        p << p_tc(0), p_tc(1)+pcom, p_tc(2);
    }

  protected:
      void _computeFrictionConeMatrix(int n, double mu, double r_pad, 
                  double x, double y, const Eigen::MatrixXd& AdT_ti){
        //---- compute U : inequality matrix

        Ut = Eigen::MatrixXd::Zero(12,6);
        Ut << 0., 0., 0., mu, mu, -1,
              0., 0., 0., mu, -mu, -1,
              0., 0., 0., -mu, mu, -1,
              0., 0., 0., -mu, -mu, -1,
              mu, mu, -1, -y,-x, -(x+y)*mu,
              mu, -mu, -1, -y, x, -(x+y)*mu,
              -mu, mu, -1, y,-x, -(x+y)*mu,
              -mu, -mu, -1, y, x, -(x+y)*mu,
              mu, mu, -1, y, x, -(x+y)*mu,
              mu, -mu, -1, y, -x, -(x+y)*mu,
              -mu, mu, -1, -y, x, -(x+y)*mu,
              -mu, -mu, -1, -y, -x, -(x+y)*mu;

        Ut = Ut*AdT_ti.transpose();

        // only z force & mx,my
        Eigen::MatrixXd Ui { {0,0,0,0,0,-1},
                          {1,1,0,0,0,-r_pad}, 
                          {-1,-1,0,0,0,-r_pad},
                          {-1,1,0,0,0,-r_pad}, 
                          {1,-1,0,0,0,-r_pad} };
        
        // std::cout<<"Ui = "<<Ui<<std::endl; 
        int nr = Ui.rows();
        int nc = Ui.cols();
        U = Eigen::MatrixXd::Zero(n*nr,n*nc);
        for(int i(0); i<n; ++i)
            U.block(i*nr, i*nc, nr, nc) = Ui;
    }
    
  public:
  // --------------------- 
  //      object data
  // ---------------------
  // Assume object frame attached to object CoM,
  // with same orientation 
  double m; // mass 
  Eigen::MatrixXd I; // Inertia in object frame
  Eigen::VectorXd p; // from tool to object frame
  // for test
  Eigen::VectorXd I1; // Inertia when assume m=1 

  // --------------------- 
  //      gripper data
  // ---------------------
  int n; // number of suction cups
  Eigen::VectorXd p_tc {{0., 0., 0.}};// tool frame to gripper center
  Eigen::VectorXd mus; // friction coefficients of each pad
  Eigen::VectorXd rpads; // radius of each suction pad 
  Eigen::MatrixXd U; // [5n X 6n] grip stability inequality matrix
  Eigen::MatrixXd Ut; // [12 x 6] slip stability inequatlity matrix 
  Eigen::MatrixXd G; // [6 X 6n] suction pad configuration matrix [AdT_it]T
  Eigen::VectorXd fs; // [6n X 1] suction force expressed in suc frames
};

class SYSTEM_DATA{
  public:
  std::vector< double > s;
  std::vector< Eigen::VectorXd > q;
  std::vector< Eigen::VectorXd > dq; // q'
  std::vector< Eigen::VectorXd > ddq; // q''

  std::vector< Eigen::VectorXd > m; // mass coeffs 
  std::vector< Eigen::VectorXd > b; // coriolis coeffs
  std::vector< Eigen::VectorXd > g; // gravity coeffs
  std::vector< Eigen::VectorXd > av; // vel coeffs
  std::vector< Eigen::VectorXd > vm2; // vel max square
  std::vector< Eigen::VectorXd > tm; // torque max  
  std::vector< Eigen::VectorXd > am; // acceleration max
  std::vector< Eigen::VectorXd > jm; // jerk max

  // linear constraints: bodynodejacobian
  // v = (Jq')*ds2
  // a = (J'q'+Jq'')*ds2 + (Jq')*dds
  std::vector< Eigen::VectorXd > ee; // ee pos
  std::vector< Eigen::VectorXd > ee_v; 
  std::vector< Eigen::VectorXd > ee_a;   

  // for grasping constraints: bodynodebodyjacob
  std::vector< Eigen::VectorXd > ee_w;
  std::vector< Eigen::VectorXd > ee_aw1;
  std::vector< Eigen::VectorXd > ee_aw2;
  std::vector< Eigen::VectorXd > ee_av1;
  std::vector< Eigen::VectorXd > ee_av2; 
  std::vector< Eigen::VectorXd > ee_grav;

  // a_g s'' + b_g s'2 + c_g < 0
  std::vector< Eigen::VectorXd > a_g;
  std::vector< Eigen::VectorXd > b_g;
  std::vector< Eigen::VectorXd > c_g;

  // Fi = m_suc s'' + b_suc s'2 + g_suc
  std::vector< Eigen::VectorXd > m_suc;
  std::vector< Eigen::VectorXd > b_suc;
  std::vector< Eigen::VectorXd > g_suc;

  // linear vel/acc limits : -1 if not activated
  std::vector< double > lvm;
  std::vector< double > lam;

  SYSTEM_DATA(){ resize(0); }
  ~SYSTEM_DATA(){}
  void resize(int n){
    s.resize(n);
    q.resize(n);
    dq.resize(n);
    ddq.resize(n);  

    m.resize(n);
    b.resize(n);
    g.resize(n);
    tm.resize(n);
    am.resize(n);
    jm.resize(n);    
    av.resize(n);
    vm2.resize(n);

    ee.resize(n);
    ee_v.resize(n);
    ee_a.resize(n);

    ee_w.resize(n);
    ee_aw1.resize(n);
    ee_aw2.resize(n);
    ee_av1.resize(n);
    ee_av2.resize(n);
    ee_grav.resize(n);

    a_g.clear();
    b_g.clear();
    c_g.clear();

    m_suc.clear();
    b_suc.clear();
    g_suc.clear();


    lvm.resize(n);
    lam.resize(n);
  }
  int getsize(){
    return q.size();
  }
  void savedata(){
    for(auto &data: s)
      rossy_utils::saveValue(data, "pathparam/s");
    for(auto &data: q)
      rossy_utils::saveVector(data, "pathparam/q");
    for(auto &data: dq)
      rossy_utils::saveVector(data, "pathparam/dq");
    for(auto &data: ddq)
      rossy_utils::saveVector(data, "pathparam/ddq");
    for(auto &data: m)
      rossy_utils::saveVector(data, "pathparam/m");
    for(auto &data: b)
      rossy_utils::saveVector(data, "pathparam/b");
    for(auto &data: g)
      rossy_utils::saveVector(data, "pathparam/g");
    for(auto &data: tm)
      rossy_utils::saveVector(data, "pathparam/tm");
    for(auto &data: am)
      rossy_utils::saveVector(data, "pathparam/am");
    for(auto &data: jm)
      rossy_utils::saveVector(data, "pathparam/jm");
    for(auto &data: av)
      rossy_utils::saveVector(data, "pathparam/av");
    for(auto &data: vm2)
      rossy_utils::saveVector(data, "pathparam/vm2");
    for(auto &data: ee)
      rossy_utils::saveVector(data, "pathparam/ee");
    for(auto &data: ee_v)
      rossy_utils::saveVector(data, "pathparam/ee_v");
    for(auto &data: ee_a)
      rossy_utils::saveVector(data, "pathparam/ee_a");
    for(auto &data: ee_w)
      rossy_utils::saveVector(data, "pathparam/ee_w"); 
    for(auto &data: ee_aw1)
      rossy_utils::saveVector(data, "pathparam/ee_aw1"); 
    for(auto &data: ee_aw2)
      rossy_utils::saveVector(data, "pathparam/ee_aw2");
    for(auto &data: ee_av1)
      rossy_utils::saveVector(data, "pathparam/ee_av1");
    for(auto &data: ee_av2)
      rossy_utils::saveVector(data, "pathparam/ee_av2");
    for(auto &data: ee_grav)
      rossy_utils::saveVector(data, "pathparam/ee_grav");
    for(auto &data: lvm)
      rossy_utils::saveValue(data, "pathparam/lvm");
    for(auto &data: lam)
      rossy_utils::saveValue(data, "pathparam/lam");
  }
};
