
#ifndef PATH_HERMITE_SPLINES_H
#define PATH_HERMITE_SPLINES_H

#include <iostream>

#include <Eigen/Dense>
#include <vector>

// spline class for TOPP(time optimal path parameterization)
// given s0, s1, ..., sN and corresponding
// x0, x1, ..., xN, u0, u1, ..., uN-1 where xk = sdot_k^2, uk=sddot_k 
// find s(t) spline parameterization

class TOPPHermiteSplines{
  public:
    TOPPHermiteSplines();
    // TOPPHermiteSplines(const double &)
    ~TOPPHermiteSplines(){};

    void initialize();
    void push_back(double p, double x);
    void compute();
    void check();

    double getPeroid();
    void getPeroids(std::vector<double> &_ts) ;
    
    double evaluate(const double & t_in);
    double evaluateFirstDerivative(const double & t_in);
    double evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);  

  private:
    std::vector<double> ps; // waypoints p
    std::vector<double> vs; // waypoints pdot = (sqrt(x))
    int n_wpts; // # of waypoints    

    double s;
    double p;
    double pdot;
    double pddot; 

    bool computed;
    // retimed timeline
    std::vector<double> ts; // timeinterval: t(i)
};


class TOPPHSpln4Vec{
  public:
    TOPPHSpln4Vec();
    ~TOPPHSpln4Vec(){};

    void initialize(int _dim);
    void push_back(const Eigen::VectorXd& P, 
                    const Eigen::VectorXd& X);
    void compute();

    Eigen::VectorXd evaluate(const double & t_in);
    Eigen::VectorXd evaluateFirstDerivative(const double & t_in);
    Eigen::VectorXd evaluateSecondDerivative(const double & t_in);

private:
    int dim; // dim of vector
    std::vector<TOPPHermiteSplines> curves;
    bool computed;
    Eigen::VectorXd output;    
}; 

#endif