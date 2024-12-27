#ifndef NATURAL_QUARTIC_SPLINES_H
#define NATURAL_QUARTIC_SPLINES_H
// Implement of Banchs, Rafael E. "Natural Quartic Spline."


#include <iostream>

#include <Eigen/Dense>
#include <vector>
#include "natural_spline_types.hpp"

class NaturalQuarticSplines{
  public:
    NaturalQuarticSplines();
    // NaturalQuarticSplines(const double &)
    ~NaturalQuarticSplines(){};

    void initialize();
    void push_back(double t, double y);
    void compute();
    void computeNatural(); 
    void computeOpt(); 
    void setType(const SplineType& type){
      quartic_spline_type=type;
    }
    

    double evaluate(const double & t_in);
    double evaluateFirstDerivative(const double & t_in);
    double evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);

  private:
    SplineType quartic_spline_type;
    std::vector<double> ts; // timelines
    std::vector<double> ys; // waypoints
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

// NQSpln4Vec : Natural Quartic Spline for Vectors
class NQSpln4Vec{
  public:
    NQSpln4Vec();
    ~NQSpln4Vec(){};

    void initialize(int _dim);
    void push_back(double t, Eigen::VectorXd Y);
    void compute();
    void setType(const SplineType& type){
      for(auto& crv: curves) crv.setType(type);
    }

    Eigen::VectorXd evaluate(const double & t_in);
    Eigen::VectorXd evaluateFirstDerivative(const double & t_in);
    Eigen::VectorXd evaluateSecondDerivative(const double & t_in);

private:
    int dim; // dim of vector
    std::vector<NaturalQuarticSplines> curves;
    bool computed;
    Eigen::VectorXd output;    
}; 

// NQSpln4Rot : Natural Quartic Spline for Rotation
class NQSpln4Rot{
  //Basic idea : do it on logarithm space
  public:
    NQSpln4Rot();
    ~NQSpln4Rot(){};

    void initialize();
    void push_back(double t, Eigen::Quaterniond q_ti);
    void compute();
    void setType(const SplineType& type){
      log_curves.setType(type);
    }

    Eigen::Quaterniond evaluate(const double & t_in);
    Eigen::Vector3d evaluateFirstDerivative(const double & t_in);
    Eigen::Vector3d evaluateSecondDerivative(const double & t_in);    

  private:
    // std::vector<NaturalQuarticSplines> curves;
    NQSpln4Vec log_curves;
    bool computed;
    bool initialized;

    Eigen::Quaterniond delq;
    Eigen::Quaterniond q_init;    
}; 

#endif