#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <io/io_utilities.hpp>
#include <iostream>

namespace rossy_utils {

// =============================================================================
// Matrix Utils
// =============================================================================
Eigen::MatrixXd skew(const Eigen::Vector3d& w);
Eigen::MatrixXd hStack(const Eigen::MatrixXd& a_, const Eigen::MatrixXd& b_);
Eigen::MatrixXd vStack(const Eigen::MatrixXd& a_, const Eigen::MatrixXd& b_);
Eigen::MatrixXd hStack(const Eigen::VectorXd& a_, const Eigen::VectorXd& b_);
Eigen::VectorXd vStack(const Eigen::VectorXd& a_, const Eigen::VectorXd& b_);
Eigen::VectorXd vStack(const Eigen::VectorXd& a_, double b_);

Eigen::MatrixXd dStack(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b);
Eigen::MatrixXd deleteRow(const Eigen::MatrixXd& a_, int row);

Eigen::VectorXd MatrixtoVector(const Eigen::MatrixXd& a);
Eigen::MatrixXd VectortoMatrix(const Eigen::VectorXd& a, int dim);
Eigen::VectorXd vector2EigenVector(const std::vector<double>& vec, int k, int l);
Eigen::VectorXd vector2EigenVector(const std::vector<double>& vec);

// void hStackConserve(Eigen::MatrixXd& a, const Eigen::MatrixXd& b);
// void hStackConserve(Eigen::VectorXd& a_, const Eigen::VectorXd& b_);
// void vStackConserve(Eigen::MatrixXd& a, const Eigen::MatrixXd& b);
// void vStackConserve(Eigen::VectorXd& a, const Eigen::VectorXd& b);
// void dStackConserve(Eigen::MatrixXd& a, const Eigen::MatrixXd& b);

double getMaxRatioValue(const Eigen::VectorXd& val, const Eigen::VectorXd& max);

Eigen::Vector3d convertQuatToExp(const Eigen::Quaterniond& q);
Eigen::VectorXd convertQuatDesToOriDes(const Eigen::Quaterniond& quat_in);
void convertQuatDesToOriDes(const Eigen::Quaterniond& quat_in,  
                            Eigen::VectorXd& ori_out);
                            
void convertIsoToVec6d(const Eigen::Isometry3d& iso_in,
                    Eigen::VectorXd& vec_out);
void convertIsoToVec7d(const Eigen::Isometry3d& iso_in,
                    Eigen::VectorXd& vec_out);


// =============================================================================
// Simple Trajectory Generator
// =============================================================================
double smooth_changing(double ini, double end, double moving_duration,
                       double curr_time);
double smooth_changing_vel(double ini, double end, double moving_duration,
                           double curr_time);
double smooth_changing_acc(double ini, double end, double moving_duration,
                           double curr_time);
void getSinusoidTrajectory(double initTime_, const Eigen::VectorXd& midPoint_,
                           const Eigen::VectorXd& amp_,
                           const Eigen::VectorXd& freq_, double evalTime_,
                           Eigen::VectorXd& p_, Eigen::VectorXd& v_,
                           Eigen::VectorXd& a_);
double smoothing(double ini, double fin, double rat);

// =============================================================================
// ETC
// =============================================================================
double bind_half_pi(double);

bool isEqual(const Eigen::VectorXd a, const Eigen::VectorXd b,
             const double threshold = 0.00001);
double CropValue(double value, double min, double max, std::string source);

Eigen::VectorXd CropVector(Eigen::VectorXd value, Eigen::VectorXd min,
                           Eigen::VectorXd max, std::string source);

Eigen::MatrixXd CropMatrix(Eigen::MatrixXd value, Eigen::MatrixXd min,
                           Eigen::MatrixXd max, std::string source);

bool isInBoundingBox(const Eigen::VectorXd& val, const Eigen::VectorXd& lb,
                     const Eigen::VectorXd& ub);

Eigen::MatrixXd GetRelativeMatrix(const Eigen::MatrixXd value,
                                  const Eigen::MatrixXd min,
                                  const Eigen::MatrixXd max);

Eigen::VectorXd GetRelativeVector(const Eigen::VectorXd value,
                                  const Eigen::VectorXd min,
                                  const Eigen::VectorXd max);

Eigen::VectorXd eulerIntegration(const Eigen::VectorXd& x,
                                 const Eigen::VectorXd& xdot, double dt);

Eigen::VectorXd doubleIntegration(const Eigen::VectorXd& q,
                                  const Eigen::VectorXd& alpha,
                                  const Eigen::VectorXd& alphad, double dt);
}  // namespace rossy_utils
