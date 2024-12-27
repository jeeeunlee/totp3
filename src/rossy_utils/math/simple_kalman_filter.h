
#ifndef SIMPLE_KALMAN_FILTER
#define SIMPLE_KALMAN_FILTER

#include <Eigen/Dense>
#include "math/pseudo_inverse.hpp"
#include <deque>

class SimpleSystemParam{
    // x(k+1) = F*x(k) + w(k), w(k)~N(0,Q)
    // z(k) = H*x(k) + v(k), v(k)~N(0,R)
    public:
        Eigen::MatrixXd F;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd H;
        Eigen::MatrixXd R;
        SimpleSystemParam(){}
        ~SimpleSystemParam(){}
};

class SimpleKalmanFilter{
public:
	SimpleKalmanFilter() { b_initialize= false;}
	~SimpleKalmanFilter() {}
	
	void initialize(const Eigen::VectorXd& x0,
                    const Eigen::MatrixXd& P0) {
        x_hat_ = x0;
        P_hat_ = P0; 
        n_ = x0.size();
        b_initialize = true;  
        n_err_change_ = 10;
        err_change_.clear();
    }

    void propagate(const Eigen::VectorXd& z, Eigen::VectorXd& xhat, void* system_data = NULL){
        if(system_data) sys_param_ = static_cast<SimpleSystemParam*>(system_data);

        xhat = x_hat_; // prev
        predict();
        update(z);        
        
        // err_change_.push_back( (x_hat_ - xhat).norm());
        err_change_.push_back( std::fabs(x_hat_(0) - xhat(0)) );
        if(err_change_.size()>n_err_change_) err_change_.pop_front();
        xhat = x_hat_;
    }

    void predict() {
        x_pre_ =  sys_param_->F * x_hat_;
        P_pre_ =  sys_param_->F * P_hat_ * sys_param_->F.transpose() + sys_param_->Q;
    }

    void update(const Eigen::VectorXd & z) {
        S_ = sys_param_->H * P_pre_ * sys_param_->H.transpose() + sys_param_->R;
        rossy_utils::pseudoInverse(S_, 0.001, S_inv_);
        K_ = P_pre_ * sys_param_->H.transpose()*S_inv_;
        y_ = z - sys_param_->H * x_pre_;

        P_hat_ = (Eigen::MatrixXd::Identity(n_,n_) - K_*sys_param_->H)*P_pre_;        
        x_hat_ = x_pre_ + K_*y_;        
    }

    double getErrorChange(){
        if(err_change_.size()==n_err_change_) {
            // double sum = 0.;
            // for(auto &val : err_change_) sum += val;
            // return sum/err_change_.size();

            double max = 0.;
            for(auto &val : err_change_) max = max>val ? max:val;
            return max;
        }
        else
            return 1e5;
    }

public:
	bool b_initialize;

private:
    SimpleSystemParam* sys_param_;
    int n_; // x dim

    Eigen::VectorXd x_hat_;    
    Eigen::VectorXd x_pre_; //predict
    Eigen::VectorXd y_; // observation error
    

    Eigen::MatrixXd P_hat_;
    Eigen::MatrixXd P_pre_;
    Eigen::MatrixXd S_; // innovation
    Eigen::MatrixXd S_inv_;
    Eigen::MatrixXd K_; // kalman gain

    // additional function
    std::deque<double> err_change_;
    int n_err_change_;   
};

#endif