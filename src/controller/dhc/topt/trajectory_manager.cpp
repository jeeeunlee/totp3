#include "controller/dhc/topt/trajectory_manager.hpp"
#include "rossy_utils/io/io_utilities.hpp"

TrajectoryManager::TrajectoryManager(){
    rossy_utils::pretty_constructor(1, "TrajectoryManager");
    use_t2q_=false;
    use_t2s_=false;
}

void TrajectoryManager::setS2QSpline(const std::vector<double>& s_list,
                        const std::vector<Eigen::VectorXd>& qwpts){
    int num_wpts = qwpts.size();
    double send = s_list[num_wpts-1];
    double s = 0.;

    spline_s2q_.initialize(qwpts[0].size());
    spline_s2q_.setType(SplineType::SAMEJERK); // NATURAL OPT SAMEACC
    for(int i(0); i<num_wpts; i++) {
        s = s_list[i] / send;
        spline_s2q_.push_back(s, qwpts[i]);
    }
    spline_s2q_.compute();

    checkSplines();
}

void TrajectoryManager::setS2QSpline(
    const std::vector<Eigen::VectorXd>& qwpts){
    int num_wpts = qwpts.size();
    double ds = 1/((double)(num_wpts-1));
    double s = 0.;

    spline_s2q_.initialize(qwpts[0].size());
    spline_s2q_.setType(SplineType::SAMEJERK); // NATURAL OPT SAMEACC
    for(int i(0); i<num_wpts; i++) {
        spline_s2q_.push_back(s, qwpts[i]);
        s = s + ds;
    }
    spline_s2q_.compute();

    checkSplines();
}

void TrajectoryManager::setT2SSpline(
        const std::vector<double>& s_list,
        const std::vector<double>& xmax_list){
    use_t2s_=true;
    use_t2q_=false;
    // set spline_t2s_
    spline_t2s_.initialize();
    ts_.clear();

    int nwpts = s_list.size();
    double s, x, u;   
    for(int i(0); i<nwpts; ++i){
        s = s_list[i];
        x = xmax_list[i];
        // 
        if(i==nwpts-1){
            u = 0.;
        }else{
            u = (xmax_list[i+1]-xmax_list[i])
                /2./(s_list[i+1]-s_list[i]); 
        }        
        spline_t2s_.push_back(s,x,u);
    }
    
    spline_t2s_.compute();
    spline_t2s_.getPeroids(ts_);
}

// if NCSpln4Vec
void TrajectoryManager::setT2QSpline(
        const std::vector<double>& s_list,
        const std::vector<double>& xmax_list){
    use_t2s_=false;
    use_t2q_=true;
    // compute time at each waypoints
    int nwpts = s_list.size();
    ts_.clear();
    
    double t=0.;
    ts_.push_back(t);
    double sdot0, sdot1;   
    for(int i(0); i<nwpts-1; ++i){
        sdot0 = sqrt(xmax_list[i]);
        sdot1 = sqrt(xmax_list[i+1]);     
        t += (s_list[i+1]-s_list[i]) / (0.5*(sdot0+sdot1)) ;        
        ts_.push_back(t);
    }    

    // set spline_t2q_
    spline_t2q_.initialize(spline_s2q_.getDim());
    spline_t2q_.setType(SplineType::ZEROVEL);
    Eigen::VectorXd q;
    for(int i(0); i<nwpts; i++) {
        q = spline_s2q_.evaluate(s_list[i]);
        spline_t2q_.push_back(ts_[i], q);
    }
    spline_t2q_.compute();    
}

// if spline_t2q_ = NVCQSpln4Vec
// void TrajectoryManager::setT2QSpline(
//         const std::vector<double>& s_list,
//         const std::vector<double>& xmax_list){
//     use_t2s_=false;
//     use_t2q_=true;

//     // compute time at each waypoints
//     int nwpts = s_list.size();
//     ts_.clear();
//     spline_t2q_.initialize(spline_s2q_.getDim());

//     double t=0.;
//     ts_.push_back(t);
//     double sdot0, sdot1, ds;
//     Eigen::VectorXd q, qdot, q1, q2;   
//     for(int i(0); i<nwpts-1; ++i){
//         sdot0 = sqrt(xmax_list[i]);
//         sdot1 = sqrt(xmax_list[i+1]);
//         ds = (s_list[i+1]-s_list[i]);
//         t += ds / (0.5*(sdot0+sdot1)) ;        
//         ts_.push_back(t);
//     }
//     for(int i(0); i<nwpts; ++i){
//         q = spline_s2q_.evaluate(s_list[i]);
//         qdot = spline_s2q_.evaluateFirstDerivative(s_list[i]);
//         qdot = qdot*sqrt(xmax_list[i]);     

//         spline_t2q_.push_back(ts_[i], q, qdot);
//     }
//     spline_t2q_.compute();    
// }

// if HQSpln4Vec
// void TrajectoryManager::setT2QSpline(
//         const std::vector<double>& s_list,
//         const std::vector<double>& xmax_list){
//     use_t2s_=false;
//     use_t2q_=true;

//     // compute time at each waypoints
//     int nwpts = s_list.size();
//     ts_.clear();
//     std::vector<double> u_list ={}; 
    
//     double t=0.;
//     ts_.push_back(t);
//     u_list.push_back(0.);
//     double sdot0, sdot1, ds;   
//     for(int i(0); i<nwpts-1; ++i){
//         sdot0 = sqrt(xmax_list[i]);
//         sdot1 = sqrt(xmax_list[i+1]);
//         ds = (s_list[i+1]-s_list[i]);     
//         t += ds / (0.5*(sdot0+sdot1)) ;        
//         ts_.push_back(t);
//         u_list.push_back( (xmax_list[i+1] - xmax_list[i]) / ( 2.*ds ) ); 
//     }
//     u_list.push_back(0.);

//     // set spline_t2q_
//     spline_t2q_.initialize(spline_s2q_.getDim());
//     Eigen::VectorXd q, qdot, qddot;
//     for(int i(0); i<nwpts; i++) {
//         q = spline_s2q_.evaluate(s_list[i]);
        
//         double sdot = sqrt(xmax_list[i]);
//         Eigen::VectorXd dq = spline_s2q_.evaluateFirstDerivative(s_list[i]);
//         qdot = dq*sdot;

//         double sddot = u_list[i];
//         Eigen::VectorXd ddq = spline_s2q_.evaluateSecondDerivative(s_list[i]);
//         qddot = dq*sddot + ddq*sdot*sdot;

//         spline_t2q_.push_back(ts_[i], q, qdot, qddot);
//     }
//     spline_t2q_.compute();    
// }

// void TrajectoryManager::setT2QSpline(
//         const std::vector<double>& t_list,
//         const std::vector<Eigen::VectorXd>& q_list){
//     assert(t_list.size() == q_list.size());
//     assert(q_list.size() > 1);
//     use_t2s_=false;
//     use_t2q_=true;

//     int nwpts = t_list.size();
//     int dim = q_list[0].size();
//     ts_ = t_list;

//     // set spline_t2q_
//     spline_t2q_.initialize(dim);
//     spline_t2q_.setType(SplineType::ZEROVEL);
//     for(int i(0); i<nwpts; i++) {
//         spline_t2q_.push_back(ts_[i], q_list[i]);
//     }
//     spline_t2q_.compute();
// }

int TrajectoryManager::evaluateTimeInterval(double t){
    assert(use_t2q_ || use_t2s_);
    // from spline_t2q_, evaluate n-th inverval
    if(use_t2q_)
        return spline_t2q_.evaluateTimeInterval(t);
    if(use_t2s_)
        return spline_t2s_.evaluateTimeInterval(t);
}

void TrajectoryManager::getCommand(double t, Eigen::VectorXd& q_cmd) {
    assert(use_t2q_ || use_t2s_);
    if(use_t2q_)
        q_cmd = spline_t2q_.evaluate(t);
    if(use_t2s_){
        double s = spline_t2s_.evaluate(t);
        q_cmd = spline_s2q_.evaluate(s);
    }        
}

void TrajectoryManager::getCommand(double t, 
                                    Eigen::VectorXd& q_cmd, 
                                    Eigen::VectorXd& qdot_cmd) {
    assert(use_t2q_ || use_t2s_);
    if(use_t2q_){
        q_cmd = spline_t2q_.evaluate(t);
        qdot_cmd = spline_t2q_.evaluateFirstDerivative(t);
    }
    if(use_t2s_){
        double s = spline_t2s_.evaluate(t);
        q_cmd = spline_s2q_.evaluate(s);

        double sdot = spline_t2s_.evaluateFirstDerivative(t);
        qdot_cmd = spline_s2q_.evaluateFirstDerivative(s) * sdot;
    }
}

void TrajectoryManager::getCommand(double t, 
                                    Eigen::VectorXd& q_cmd, 
                                    Eigen::VectorXd& qdot_cmd,
                                    Eigen::VectorXd& qddot_cmd) {
    assert(use_t2q_ || use_t2s_);
    if(use_t2q_){
        q_cmd = spline_t2q_.evaluate(t);
        qdot_cmd = spline_t2q_.evaluateFirstDerivative(t);
        qddot_cmd = spline_t2q_.evaluateSecondDerivative(t);
    }
    if(use_t2s_){
        double s = spline_t2s_.evaluate(t);
        q_cmd = spline_s2q_.evaluate(s);

        double sdot = spline_t2s_.evaluateFirstDerivative(t);
        Eigen::VectorXd dq = spline_s2q_.evaluateFirstDerivative(s);
        qdot_cmd = dq*sdot;

        double sddot = spline_t2s_.evaluateSecondDerivative(t);
        Eigen::VectorXd ddq = spline_s2q_.evaluateSecondDerivative(s);
        qddot_cmd = dq*sddot + ddq*sdot*sdot;
    }
}

void TrajectoryManager::redistQwptsPureNormDist(
        const std::vector<Eigen::VectorXd>& qwpts0,
        std::vector<Eigen::VectorXd>& qwpts ){
    int num_wpts = qwpts0.size();
    double s(0.);
    Eigen::VectorXd delq;
    std::vector<double> slist;
    slist.push_back(s);
    for(int i(0); i<num_wpts-1; i++){
        delq = qwpts0[i+1] - qwpts0[i];
        s += delq.norm();
        slist.push_back(s);
    }
    
    double ds =  1./((double)num_wpts-1.);
    double alpha, ssum, s_uni;    
    bool monoincrease = false;
    ssum = slist[num_wpts-1];  
    for(int i(0); i<num_wpts; i++) {
        slist[i] = (slist[i]/ssum);
    }    

    spline_s2q_.initialize(qwpts0[0].size());
    spline_s2q_.setType(SplineType::SAMEJERK); 
    for(int i(0); i<num_wpts; i++) {
        spline_s2q_.push_back(slist[i], qwpts0[i]);
    }
    spline_s2q_.compute();

    qwpts.clear();
    int num_new_wpts = num_wpts;
    num_new_wpts = std::min(num_new_wpts, MAX_NWPTS_);
    num_new_wpts = std::max(num_new_wpts, MIN_NWPTS_);
    Eigen::VectorXd q_uni;
    for(int i(0); i<num_new_wpts; i++) {  
        s = ((double)i/(double)(num_new_wpts-1));
        q_uni = spline_s2q_.evaluate(s);
        qwpts.push_back(q_uni);
    }
}

void TrajectoryManager::redistQwptsNormDist(
        const std::vector<Eigen::VectorXd>& qwpts0,
        std::vector<Eigen::VectorXd>& qwpts ){
    int num_wpts = qwpts0.size();
    double s(0.);
    Eigen::VectorXd delq;
    std::vector<double> slist;
    slist.push_back(s);
    for(int i(0); i<num_wpts-1; i++){
        delq = qwpts0[i+1] - qwpts0[i];
        s += delq.norm();
        slist.push_back(s);
    }
    
    double ds =  1./((double)num_wpts-1.);
    double alpha, ssum, s_uni;    
    bool monoincrease = false;
    while(!monoincrease){
        ssum = slist[num_wpts-1];  
        monoincrease = true;
        for(int i(0); i<num_wpts; i++) {
            s_uni = ((double) i)*ds;
            alpha = sin(M_PI*s_uni);
            slist[i] = alpha*(slist[i]/ssum) + (1.-alpha)*s_uni;
            if(i>0 && slist[i]<slist[i-1])
                monoincrease = false;
        }
    }

    spline_s2q_.initialize(qwpts0[0].size());
    spline_s2q_.setType(SplineType::SAMEJERK); 
    for(int i(0); i<num_wpts; i++) {
        spline_s2q_.push_back(slist[i], qwpts0[i]);
    }
    spline_s2q_.compute();

    qwpts.clear();
    int num_new_wpts = num_wpts;
    num_new_wpts = std::min(num_new_wpts, MAX_NWPTS_);
    num_new_wpts = std::max(num_new_wpts, MIN_NWPTS_);
    Eigen::VectorXd q_uni;
    for(int i(0); i<num_new_wpts; i++) {  
        s = ((double)i/(double)(num_new_wpts-1));
        q_uni = spline_s2q_.evaluate(s);
        qwpts.push_back(q_uni);
    }
}

void TrajectoryManager::redistQwpts1st(
    const std::vector<Eigen::VectorXd>& qwpts0,
    const Eigen::VectorXd& qvelmax,
    std::vector<Eigen::VectorXd>& qwpts ){
    int num_wpts = qwpts0.size();
    Eigen::VectorXd dq, qvelmaxinv;
    qvelmaxinv = qvelmax.cwiseInverse();

    double s, qarclength = 0.;
    std::vector<double> qarclengths;
    qarclengths.push_back(0.);

    for(int i(1); i<num_wpts; i++) {
        dq = qwpts0[i]-qwpts0[i-1];
        dq = dq.cwiseProduct(qvelmaxinv);
        dq = dq.cwiseAbs();
        qarclength += std::max(1e-5, dq.maxCoeff());
        qarclengths.push_back(qarclength);
    }
    
    spline_s2q_.initialize(qwpts[0].size());
    spline_s2q_.setType(SplineType::SAMEJERK);   
    for(int i(0); i<num_wpts; i++) {        
        s = qarclengths[i]/qarclength;
        spline_s2q_.push_back(s, qwpts[i]);
    }
    spline_s2q_.compute();

    qwpts.clear();
    int num_new_wpts = 2*num_wpts;
    num_new_wpts = std::min(num_new_wpts, MAX_NWPTS_);
    num_new_wpts = std::max(num_new_wpts, MIN_NWPTS_);
    Eigen::VectorXd q_uni;
    for(int i(0); i<num_new_wpts; i++) {  
        s = ((double)i/(double)(num_new_wpts-1));
        q_uni = spline_s2q_.evaluate(s);
        qwpts.push_back(q_uni);
    }
}

void TrajectoryManager::redistQwpts2nd(
        std::vector<Eigen::VectorXd>& qwpts ){

    // Assume T2Q spline is set 
    // Redistribute the points uniformly over time
    
    qwpts.clear();
    int nwpts = ts_.size();
    Eigen::VectorXd q_cmd;
    double tstep = 0.05;
    double t;

    int ts_size = ts_.size();    
    std::vector<double> ts_new;
    for(int i(0); i<ts_size; ++i){
        t = ts_[i];
        q_cmd = spline_t2q_.evaluate(t); 
        qwpts.push_back(q_cmd);
        ts_new.push_back(t);
        if(i<ts_size-1){
            int d = (int) ((ts_[i+1]-ts_[i]) / tstep);
            for(int di(0); di<d; di++){
                t = ts_[i] + (((double)(di+1)/(double)(d+1)))*(ts_[i+1]-ts_[i]);
                q_cmd = spline_t2q_.evaluate(t); 
                qwpts.push_back(q_cmd);
                ts_new.push_back(t);
            }            
        }
    }
    
    if(ts_new.size()>MAX_NWPTS_){
        qwpts.clear();
        ts_new.clear();
        tstep = ts_.back() / ((double)(MAX_NWPTS_-1));
        for(int i(0); i<MAX_NWPTS_; ++i){    
            t = tstep * (double)i;        
            q_cmd = spline_t2q_.evaluate(t);
            qwpts.push_back(q_cmd);
            ts_new.push_back(t);            
        }
    }
}

void TrajectoryManager::redistQwptsCritical(
        std::vector<Eigen::VectorXd>& qwpts ){
    // Assume T2Q spline is set 
    // add points around max vel/acc/jerk

    qwpts.clear();
    int nwpts = ts_.size();
    Eigen::VectorXd q, qdot, qddot;

    // q = spline_t2q_.evaluate(t)
    // qdot = spline_t2q_.evaluateFirstDerivative(t)
    // qddot = spline_t2q_.evaluateSecondDerivative(t)
}

void TrajectoryManager::checkSplines(){
    // check s(t)
    // spline_t2s_.evaluate(t);

    // check q(s)
    double s=0.;
    double ds=0.001;
    Eigen::VectorXd q;
    while(s<1.){
        s+= ds;
        rossy_utils::saveValue(s, "spline/s2q_s");
        q = spline_s2q_.evaluate(s);
        rossy_utils::saveVector(q, "spline/s2q_q");
        q = spline_s2q_.evaluateFirstDerivative(s);
        rossy_utils::saveVector(q, "spline/s2q_dq");
        q = spline_s2q_.evaluateSecondDerivative(s);
        rossy_utils::saveVector(q, "spline/s2q_ddq");
    }    
}