#ifndef HERMITE_QUINTIC_SPLINES_H
#define HERMITE_QUINTIC_SPLINES_H


#include <iostream>

#include <Eigen/Dense>
#include <vector>

class HermiteQuinticSplines{
  public:
    HermiteQuinticSplines();
    ~HermiteQuinticSplines(){};

    void initialize();
    void push_back(double t, double y,
                  double ydot, double yddot);
    void compute();

    double evaluate(const double & t_in);
    double evaluateFirstDerivative(const double & t_in);
    double evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);

  private:
    std::vector<double> ts; // timelines
    std::vector<double> ys; // waypoints
    std::vector<double> dys; // waypoints
    std::vector<double> ddys; // waypoints
    int n_wpts; // # of waypoints  

    std::vector<Eigen::VectorXd> coeffs;

    bool computed;

};

// HQSpln4Vec : hermite quintic Spline for Vectors
class HQSpln4Vec{
  public:
    HQSpln4Vec();
    ~HQSpln4Vec(){};

    void initialize(int _dim);
    void push_back(double t, 
                  const Eigen::VectorXd& Y,
                  const Eigen::VectorXd& Ydot, 
                  const Eigen::VectorXd& Yddot);
    void compute();

    Eigen::VectorXd evaluate(const double & t_in);
    Eigen::VectorXd evaluateFirstDerivative(const double & t_in);
    Eigen::VectorXd evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);
private:
    int dim; // dim of vector
    std::vector<HermiteQuinticSplines> curves;
    bool computed;
    Eigen::VectorXd output;    
}; 



#endif