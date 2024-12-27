#ifndef NATURAL_CUBIC_SPLINES_H
#define NATURAL_CUBIC_SPLINES_H

#include <iostream>

#include <Eigen/Dense>
#include <vector>
#include "natural_spline_types.hpp"

class NaturalCubicSplines{
  public:
    NaturalCubicSplines();
    // NaturalCubicSplines(const double &)
    ~NaturalCubicSplines(){};

    void initialize();
    void push_back(double t, double y);
    void compute();
    void computeNatural();
    void computeSameJerk();
    void computeSameAcc();
    void computeZeroVelClamped();
    void computeOpt();
    void setType(const SplineType& type){
      cubic_spline_type=type;
    }
    int getNumWpts(){return n_wpts;}
    

    double evaluate(const double & t_in);
    double evaluateFirstDerivative(const double & t_in);
    double evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);

  private:
    SplineType cubic_spline_type;
    std::vector<double> ts; // timelines
    std::vector<double> ys; // waypoints
    int n_wpts; // # of waypoints    

    double s;
    double sdot;
    double sddot; 

    bool computed;
    std::vector<double> hs; // timeinterval: t(i+1)-t(i)
    std::vector<double> zs; // spline parmeters Az=u

};

// NCSpln4Vec : Natural Cubic Spline for Vectors
class NCSpln4Vec{
  public:
    NCSpln4Vec();
    ~NCSpln4Vec(){};

    void initialize(int _dim);
    void push_back(double t, Eigen::VectorXd Y);
    void compute();
    void setType(const SplineType& type){
      for(auto& crv: curves) crv.setType(type);
    }
    int getDim(){return dim;}
    int getNumWpts(){return n_wpts;}

    Eigen::VectorXd evaluate(const double & t_in);
    Eigen::VectorXd evaluateFirstDerivative(const double & t_in);
    Eigen::VectorXd evaluateSecondDerivative(const double & t_in);
    int evaluateTimeInterval(const double & t_in);

private:
    int dim; // dim of vector
    int n_wpts; // num of waypoints
    std::vector<NaturalCubicSplines> curves;
    bool computed;
    Eigen::VectorXd output;    
}; 

// NCSpln4Rot : Natural Cubic Spline for Rotation
class NCSpln4Rot{
  //Basic idea : do it on logarithm space
  public:
    NCSpln4Rot();
    ~NCSpln4Rot(){};

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
    // std::vector<NaturalCubicSplines> curves;
    NCSpln4Vec log_curves;
    bool computed;
    bool initialized;

    Eigen::Quaterniond delq;
    Eigen::Quaterniond q_init;    
}; 

#endif