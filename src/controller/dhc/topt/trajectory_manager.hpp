#pragma once


#include "rossy_utils/math/natural_quartic_splines.hpp"
#include "rossy_utils/math/natural_cubic_splines.hpp"
#include "rossy_utils/math/path_paramerization_splines.hpp"
#include "rossy_utils/math/path_hermite_splines.hpp"
#include "rossy_utils/math/hermite_quintic_splines.hpp"
#include "rossy_utils/math/natural_vcquartic_splines.hpp"

#include <Eigen/Dense>

class TrajectoryManager{
    public:
        TrajectoryManager();
        ~TrajectoryManager(){}

        void setS2QSpline(const std::vector<double>& s_list,
                        const std::vector<Eigen::VectorXd>& qwpts);
        void setS2QSpline(const std::vector<Eigen::VectorXd>& qwpts);
        void setT2SSpline(const std::vector<double>& s_list,
                        const std::vector<double>& xmax_list);
        void setT2QSpline(const std::vector<double>& s_list,
                        const std::vector<double>& xmax_list);
        // void setT2QSpline(const std::vector<double>& t_list,
        //                 const std::vector<Eigen::VectorXd>& q_list);        
        
        void getCommand(double t, Eigen::VectorXd& q_cmd);
        void getCommand(double t, 
                        Eigen::VectorXd& q_cmd, 
                        Eigen::VectorXd& qdot_cmd);
        void getCommand(double t, 
                        Eigen::VectorXd& q_cmd, 
                        Eigen::VectorXd& qdot_cmd,
                        Eigen::VectorXd& qddot_cmd);
        double getMotionPeriod(){ return ts_.back(); }
        void getMotionPeriods(std::vector<double> &_ts){  
                            _ts = ts_; }
        int evaluateTimeInterval(double t);

        void redistQwptsPureNormDist(const std::vector<Eigen::VectorXd>& qwpts0,
                                std::vector<Eigen::VectorXd>& qwpts );
        void redistQwptsNormDist(const std::vector<Eigen::VectorXd>& qwpts0,
                                std::vector<Eigen::VectorXd>& qwpts );
        void redistQwpts1st(const std::vector<Eigen::VectorXd>& qwpts0,
                            const Eigen::VectorXd& qvelmax,
                            std::vector<Eigen::VectorXd>& qwpts );
        void redistQwpts2nd(std::vector<Eigen::VectorXd>& qwpts );
        void redistQwptsCritical(std::vector<Eigen::VectorXd>& qwpts );

        // check spline units
        void checkSplines();

    public:
        NCSpln4Vec spline_s2q_;
        NCSpln4Vec spline_t2q_;
        // HQSpln4Vec spline_t2q_;
        // NVCQSpln4Vec spline_t2q_;
        std::vector<double> ts_;

        TOPPSplines spline_t2s_;
        bool use_t2q_;
        bool use_t2s_;

        const int MAX_NWPTS_ = 101;
        const int MIN_NWPTS_ = 11;
};
