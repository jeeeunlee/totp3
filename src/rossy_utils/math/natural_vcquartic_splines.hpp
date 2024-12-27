#ifndef NATURAL_QUARTIC_SPLINES_VC_H
#define NATURAL_QUARTIC_SPLINES_VC_H
// "Natural Quartic Spline with velocity condition"


#include <iostream>

#include <Eigen/Dense>
#include <vector>


class NaturalVCQuarticSplines{
  public:
    NaturalVCQuarticSplines();
    // NaturalVCQuarticSplines(const double &)
    ~NaturalVCQuarticSplines(){};

    void initialize();
    void push_back(double t, double y, double ydot);
    void compute();    

    double evaluate(const double & t_in);
    double evaluateFirstDerivative(const double & t_in);
    double evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);

  private:    
    std::vector<double> ts; // timelines
    std::vector<double> ys; // waypoints
    std::vector<double> dys; // waypoints derivitives
    int n_wpts; // # of waypoints    

    double s;
    double sdot;
    double sddot; 

    bool computed;
    std::vector<double> hs; // timeinterval: t(i+1)-t(i)
    std::vector<double> zs; // spline parmeters Az=u

    std::vector<double> Cs; // coeff Cs; (t-t[i])
    std::vector<double> Ds; // coeff Ds; (t-t[i])
    std::vector<double> Es; // coeff Es; (t[i+1]-t)

};

// NVCQSpln4Vec : Natural Quartic Spline for Vectors
class NVCQSpln4Vec{
  public:
    NVCQSpln4Vec();
    ~NVCQSpln4Vec(){};

    void initialize(int _dim);
    void push_back(double t, 
            const Eigen::VectorXd& Y, 
            const Eigen::VectorXd& YDot);
    void compute();

    Eigen::VectorXd evaluate(const double & t_in);
    Eigen::VectorXd evaluateFirstDerivative(const double & t_in);
    Eigen::VectorXd evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);
private:
    int dim; // dim of vector
    std::vector<NaturalVCQuarticSplines> curves;
    bool computed;
    Eigen::VectorXd output;    
}; 

#endif