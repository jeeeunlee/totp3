#include "rossy_utils/math/lp_solver.hpp"
#include "controller/dhc/user_command.hpp"

class LPSolver;
class TrajectoryManager;
class Clock;

struct InequalData{
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
};
typedef std::array<InequalData,3> InequalDataList;
// let kk:k+1, kpre:k-1
class ToptSolver{
  public:
    ToptSolver(int dim);
    ~ToptSolver(){};

    bool solve(const SYSTEM_DATA &sysdata,
              const GRIP_DATA &gripdata,
              TrajectoryManager* traj,
              bool thirdorder = true);

    void resolve(double planning_time, 
                const GRIP_DATA &gripdata,
                TrajectoryManager* traj);    

  private:
    void get1stlimit(int k, InequalData& constraints);
    void get2ndlimit(int k, InequalData& constraints);    
    void addgrasp2ndlimit(int k, InequalData& constraints);
    void get3rdlimit(int k, 
                    const std::vector<double>& x0_list,
                    InequalData& constraints );
    

    double getFeasibleX(int k); 
    double getControllableX(int k, double xmax_c_kk);
    double getReachableXMax(int k, double xmax_c_k, double xmax_r_kpre);

    void initializeSets();
    void initializeDimensions();
    void updateGripdataToSysdata();

    void solveTOPPRA0();
    bool solveTOPPRA(int idx_curr=0);
    void solveTOPP3(const std::vector<double>& x0_list,
                      int idx_curr=0);
    InequalDataList buildTOPP3Ineq(
        const std::vector<double>& x0_list, int k, int h,  double alpha=1.);
    void updateTOPP3Ineq(        
        const std::vector<double>& x0_list,
        int k, int h, InequalDataList& TOPP3Ineq, double alpha=1.);
    Eigen::VectorXd getCostCoeffs(
        const std::vector<double>& x0_list, int k, int h);
    
    // check functions
    double checkLimits(const std::vector<double>& x0_list);
    void checkForcesAtSuction(const std::vector<double>& x0_list);
    Eigen::VectorXd computeJerk(
        int k, const std::vector<double>& x0_list);
    double estimateMotionPeriod(const std::vector<double>& x0_list);

  protected:
    bool grasping_activated_;
    bool lin_vel_check_activated_;
    bool lin_acc_check_activated_;
    int dim_;
    int dimIneq1_;
    int dimIneq2_;
    int dimIneq3_;
    const double MIN_SDOT_ = 1e-5; 
    std::vector<double> feasible_xmax_;
    std::vector<double> controllable_xmax_;
    std::vector<double> reachable_xmax_;
    std::vector<double> trackable_xmax_;

    SYSTEM_DATA sysdata_;
    GRIP_DATA gripdata_;
    LPSolver* lpsolver_;
    Clock* clock_;
};