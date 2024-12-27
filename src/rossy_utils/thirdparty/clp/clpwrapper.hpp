#include <coin/ClpSimplex.hpp>
#include <coin/CoinPackedMatrix.hpp>
#include <Eigen/Dense>

class LPSolver{
  public:
    LPSolver();
    ~LPSolver();
    // min  f'x 
    // s.t. Ax < b
    double solve(const Eigen::VectorXd &f,
                const Eigen::MatrixXd &A,
                const Eigen::VectorXd &b,
                Eigen::VectorXd &x);

    // min  f'x
    // s.t. vlb < x  < vub
    //       lb < Ax < ub
    double solve(const Eigen::VectorXd &f,
                const Eigen::MatrixXd &A,
                const Eigen::VectorXd &lb,
                const Eigen::VectorXd &ub,
                const Eigen::VectorXd &vlb,
                const Eigen::VectorXd &vub,
                Eigen::VectorXd &x);

    CoinPackedMatrix from_EigenMatrix(const Eigen::MatrixXd& emat);

  private:
    ClpSimplex* model_;

};