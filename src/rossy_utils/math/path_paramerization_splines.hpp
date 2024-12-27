
#ifndef PATH_PARAMETERIZATION_SPLINES_H
#define PATH_PARAMETERIZATION_SPLINES_H

#include <iostream>

#include <Eigen/Dense>
#include <vector>

// spline class for TOPP(time optimal path parameterization)
// given s0, s1, ..., sN and corresponding
// x0, x1, ..., xN, u0, u1, ..., uN-1 where xk = sdot_k^2, uk=sddot_k 
// find s(t) spline parameterization

class TOPPSplines{
  public:
    TOPPSplines();
    // TOPPSplines(const double &)
    ~TOPPSplines(){};

    void initialize();
    void push_back(double s, double x, double u);
    void compute();
    void check();

    double getPeroid();
    void getPeroids(std::vector<double> &_ts) ;
    
    double evaluate(const double & t_in);
    double evaluateFirstDerivative(const double & t_in);
    double evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);  

  private:
    std::vector<double> ss; // waypoints s
    std::vector<double> vs; // waypoints sdot = (sqrt(x))
    std::vector<double> us; // waypoints sddot
    int n_wpts; // # of waypoints    

    double s;
    double sdot;
    double sddot; 

    bool computed;
    // retimed timeline
    std::vector<double> ts; // timeinterval: t(i)

    // polynomial coeff
    // si(t) = si + sqrt(xi)(t-ti) + 0.5*ui*(t-ti)^2 
    //          + ai*()^3 + bi*()^4
    std::vector<double> as;
    std::vector<double> bs;


};


#endif