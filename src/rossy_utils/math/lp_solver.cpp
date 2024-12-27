
#include <math/lp_solver.hpp>
#include "sdlp/sdlp.hpp"
#include "clp/clpwrapper.hpp"

// solve LP, linprog(f,A,b)
// min f'x s.t. Ax<=b

namespace rossy_utils {

double linprognd(const Eigen::VectorXd& f,
                const Eigen::MatrixXd& A,
                const Eigen::VectorXd& b,
                Eigen::VectorXd& x){

    LPSolver* sol_instance = new LPSolver();

    double ret = sol_instance->solve(f,A,b,x);

    delete sol_instance;
    return ret;
}

double linprog2d(const Eigen::VectorXd& f2d,
                const Eigen::MatrixXd& A,
                const Eigen::VectorXd& b,
                Eigen::VectorXd& x2d){
    // assert()
    Eigen::Matrix<double, 2, 1> f_ = f2d;
    Eigen::Matrix<double, 2, 1> x_;
    Eigen::Matrix<double, -1, 2> A_ = A;

    double ret = sdlp::linprog<2>(f_, A_, b, x_);
    x2d=x_.col(0);
    return ret;
}

double linprog1d(const double& f,
            const Eigen::VectorXd& a,
            const Eigen::VectorXd& b,
            double& x) {
    // assert()
    Eigen::Matrix<double, 1, 1> fv = Eigen::VectorXd::Constant(1, f);
    Eigen::Matrix<double, 1, 1> xv = Eigen::VectorXd::Zero(1);
    Eigen::Matrix<double, -1, 1> A(a.size(), 1);
    A.col(0) = a;

    double ret = sdlp::linprog<1>(fv, A, b, xv);
    x = xv(0,0);
    return ret;
}
} //namespace rossy_utils
