
#ifndef LOW_PASS_FILTER_2ND
#define LOW_PASS_FILTER_2ND

#include <Eigen/Dense>

// 2nd order low pass filter
class LowPassFilter2{
public:
	LowPassFilter2(){
        zeta = 0.9;
        Ts=0.001;
        input_dim=0;
        b_initialized = false;
    }
	~LowPassFilter2() {}
	
	void initialize(int idim, double cutoff) {
        cut_off_frequency=cutoff;
        double Wn = cut_off_frequency*2.*M_PI;
        cDen[0] = 4.0+4.0*zeta*Ts*Wn + Ts*Wn*Ts*Wn;
		cDen[1] = 2.0*Ts*Wn*Ts*Wn - 8.0;
		cDen[2] = 4.0-4.0*zeta*Ts*Wn + Ts*Wn*Ts*Wn;
		cNum[0] = Ts*Ts*Wn*Wn;
		cNum[1] = 2.0*Ts*Ts*Wn*Wn;
		cNum[2] = Ts*Ts*Wn*Wn;
        input_dim = idim;
        filtered_val_0 = Eigen::VectorXd::Zero(input_dim);
        filtered_val_1 = Eigen::VectorXd::Zero(input_dim);
        filtered_val_2 = Eigen::VectorXd::Zero(input_dim);
        input_val_0 = Eigen::VectorXd::Zero(input_dim);
        input_val_1 = Eigen::VectorXd::Zero(input_dim);
        input_val_2 = Eigen::VectorXd::Zero(input_dim);
        b_initialized = true;
    }
	Eigen::VectorXd update(const Eigen::VectorXd & s_in){
        if(!b_initialized){
            std::cout<<"LowPassFilter2 didn't initialized"<< std::endl;
            initialize(s_in.size(), 30.);
        }
        input_val_0 = s_in;
        filtered_val_0 = ( cNum[0]*input_val_0 + cNum[1]*input_val_1 + cNum[2]*input_val_2
                            - cDen[1]*filtered_val_1 - cDen[2]*filtered_val_2) / cDen[0];
        
        filtered_val_2 = filtered_val_1;
        filtered_val_1 = filtered_val_0;
        input_val_2 = input_val_1;
        input_val_1 = input_val_0;

        return filtered_val_0;
    }
public:
	double cut_off_frequency;
    double zeta;
    double Ts;

private:
    int input_dim;
    bool b_initialized;

    Eigen::Vector3d cDen; // denominator coeff
    Eigen::Vector3d cNum; // numerator coeff

    Eigen::VectorXd filtered_val_0;
    Eigen::VectorXd filtered_val_1;
    Eigen::VectorXd filtered_val_2;

    Eigen::VectorXd input_val_0;
    Eigen::VectorXd input_val_1;
    Eigen::VectorXd input_val_2;
};

#endif