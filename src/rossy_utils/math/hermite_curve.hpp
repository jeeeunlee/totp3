#ifndef ALM_HERMITE_CURVE_H
#define ALM_HERMITE_CURVE_H

#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>
#include <vector>


// returns a hermite interpolation (cubic) of the boundary conditions for a given s \in [0,1].

class HermiteCurve{
public:
	HermiteCurve();
	HermiteCurve(const double & start_pos, const double & start_vel, 
				 const double & end_pos, const double & end_vel, const double & duration);
	~HermiteCurve();
	double evaluate(const double & t_in);
	double evaluateFirstDerivative(const double & t_in);
	double evaluateSecondDerivative(const double & t_in);

private:
	double p1;
	double v1;
	double p2;
	double v2;

	double t_dur;

	double s_;

	// by default clamps within 0 and 1.
	double clamp(const double & t_in, double lo = 0.0, double hi = 1.0);

};



class HermiteCurveVec{
public:
	HermiteCurveVec();
	HermiteCurveVec(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, 
				   const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const double & duration);
	~HermiteCurveVec();
	
	void initialize(const Eigen::VectorXd & start_pos, const Eigen::VectorXd & start_vel, 
					const Eigen::VectorXd & end_pos, const Eigen::VectorXd & end_vel, const double & duration);
	Eigen::VectorXd evaluate(const double & t_in);
	Eigen::VectorXd evaluateFirstDerivative(const double & t_in);
	Eigen::VectorXd evaluateSecondDerivative(const double & t_in);

private:
	Eigen::VectorXd p1;
	Eigen::VectorXd v1;
	Eigen::VectorXd p2;
	Eigen::VectorXd v2;

	double t_dur;

	std::vector<HermiteCurve> curves;
 	Eigen::VectorXd output;
};

class HermiteQuaternionCurve{
public:
	HermiteQuaternionCurve();
	HermiteQuaternionCurve(const Eigen::Quaterniond & quat_start, 
						   const Eigen::Vector3d & angular_velocity_start,
						   const Eigen::Quaterniond & quat_end, 
						   const Eigen::Vector3d & angular_velocity_end,
						   double duration);
	~HermiteQuaternionCurve();

	void initialize(const Eigen::Quaterniond& quat_start,
					const Eigen::Vector3d& angular_velocity_start,
					const Eigen::Quaterniond& quat_end,
					const Eigen::Vector3d& angular_velocity_end,
					double duration);

	// All values are expressed in "world frame"
	void evaluate(const double & t_in, Eigen::Quaterniond & quat_out);
	void getAngularVelocity(const double & t_in, Eigen::Vector3d & ang_vel_out);
	void getAngularAcceleration(const double & t_in, Eigen::Vector3d & ang_acc_out);

private:
	double t_dur; // time duration

	Eigen::Quaterniond qa; // Starting quaternion
	Eigen::Vector3d omega_a; // Starting Angular Velocity
	Eigen::Quaterniond qb; // Ending quaternion
	Eigen::Vector3d omega_b; // Ending Angular velocity

	void initialize_data_structures();
	HermiteCurveVec theta_ab; // so3 
	Eigen::Quaterniond delq;


	///////////////////////////////////////

	Eigen::AngleAxisd omega_a_aa; // axis angle representation of omega_a
	Eigen::AngleAxisd omega_b_aa; // axis angle representation of omega_b


	void computeBasis(const double & t_in); // computes the basis functions
	void computeOmegas();

	Eigen::Quaterniond q0; // quat0
	Eigen::Quaterniond q1; // quat1
	Eigen::Quaterniond q2; // quat1
	Eigen::Quaterniond q3; // quat1

	double b1; // basis 1
	double b2; // basis 2
	double b3; // basis 3

	double bdot1; // 1st derivative of basis 1
	double bdot2; // 1st derivative of basis 2
	double bdot3; // 1st derivative of basis 3

	double bddot1; // 2nd derivative of basis 1
	double bddot2; // 2nd derivative of basis 2
	double bddot3; // 2nd derivative of basis 3

	Eigen::Vector3d omega_1;
	Eigen::Vector3d omega_2;
	Eigen::Vector3d omega_3;

	Eigen::AngleAxisd omega_1aa;
	Eigen::AngleAxisd omega_2aa;
	Eigen::AngleAxisd omega_3aa;

	// Allocate memory for quaternion operations
	Eigen::Quaterniond qtmp1;
	Eigen::Quaterniond qtmp2;
	Eigen::Quaterniond qtmp3;



	void printQuat(const Eigen::Quaterniond & quat);

};

#endif