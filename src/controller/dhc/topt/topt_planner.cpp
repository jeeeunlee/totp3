#include "controller/dhc/topt/topt_planner.hpp"
#include "controller/dhc/topt/trajectory_manager.hpp"
#include "controller/dhc/topt/topt_solver.hpp"
#include "controller/my_robot_system/robot_system.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "rossy_utils/math/liegroup_utilities.hpp"
#include "controller/dhc/topt/robot_manager.hpp"
#include "cdd/src/Polytope.h"

// Time Optimal Trajectory Planner
ToptPlanner::ToptPlanner(RobotSystem* _robot, 
                        int _robot_type,
                        int _link_idx,
                        TrajectoryManager* _topt_traj) 
    :Planner(_robot), robot_type_(_robot_type), 
    link_idx_(_link_idx), threshold_pinv_(0.2) {
    rossy_utils::pretty_constructor(1, "ToptPlanner");

    n_dof_ = robot_->getNumDofs();
    robot_temp_ = new RobotSystem(*robot_);
    topt_solver_ = new ToptSolver( n_dof_ );
    switch(robot_type_){        
        case ROBOT_TYPE::RS020N:
        default:
            robot_manager_ = 
            new RS020NManager(robot_temp_, link_idx_, threshold_pinv_);
        break;
    }    
    topt_traj_ = _topt_traj;
    // grip_data_ = DEX_GRIPPER_V3();
    // set default jerk limit value as 1000
    vel_limit_ = robot_temp_->GetVelocityUpperLimits();
    acc_limit_ = Eigen::VectorXd::Constant(n_dof_, 6.);     
    jerk_limit_ = Eigen::VectorXd::Constant(n_dof_, 1000.); 
}

ToptPlanner::~ToptPlanner() {
    delete robot_temp_;
    delete topt_solver_;
}

void ToptPlanner::setGripData(const GRIP_DATA &gripdata){
    grip_data_ = gripdata;
}
void ToptPlanner::setVelLimit(const Eigen::VectorXd &vm){
    rossy_utils::pretty_print(vm, std::cout, "setVelLimit");
    vel_limit_ = vm; 
} 
void ToptPlanner::setAccLimit(const Eigen::VectorXd &am){
    rossy_utils::pretty_print(am, std::cout, "setAccLimit");
    acc_limit_ = am; 
}
void ToptPlanner::setJerkLimit(const Eigen::VectorXd &jm){
    rossy_utils::pretty_print(jm, std::cout, "setJerkLimit");
    jerk_limit_ = jm; 
}

bool ToptPlanner::rePlanning(){

    double replanning_time = current_time_-start_time_;
    std::cout<<"DHC: replanning time = "<<current_time_ << "-" 
            << start_time_ << "=" << replanning_time << std::endl;
    topt_solver_->resolve( replanning_time,  grip_data_, topt_traj_);
    
    return true;
}

void ToptPlanner::preProcessingCommand(
        PLANNING_COMMAND* planning_cmd,
        int redist_mode) {

    // make sure the right commands input
    planning_cmd->acc_percentage =
        std::min(1., planning_cmd->acc_percentage);
    planning_cmd->acc_percentage =
        std::max(planning_cmd->acc_percentage, 0.2);

    planning_cmd->dec_percentage =
        std::min(1., planning_cmd->dec_percentage);
    planning_cmd->dec_percentage =
        std::max(planning_cmd->dec_percentage, 0.2);
    
    Eigen::VectorXd q0 = robot_->getQ();

    // 1. generate qwpt based on given wpts
    // if waypoints are given cartesian
    // remapping points by solving IK (catesian -> joint) 
    if( planning_cmd->cartesian_path.size() > 0 &&
        planning_cmd->joint_path.size() == 0 ) {
        solveIK(q0,
                planning_cmd->cartesian_path,
                planning_cmd->joint_path);
    }

    // TODO: exception: no path fed 
    if(planning_cmd->joint_path.size() == 0){
        std::cout<<"EXCEPTIION!!:: no path fed "<<std::endl;
        planning_cmd->joint_path.push_back(q0);
        planning_cmd->joint_path.push_back(q0);
        planning_cmd->joint_path.push_back(q0);
        return;
    }
        
    
    Eigen::VectorXd q1 = planning_cmd->joint_path[0];
    if((q1-q0).norm() < 1e-5 && 
        planning_cmd->joint_path.size()>2 ){
        // replace the first position to q0
        planning_cmd->joint_path[0] = q0; 
    }else {
        // add current position in joint path
        planning_cmd->joint_path.insert(
            planning_cmd->joint_path.begin(), 0.5*(q0+q1));
        planning_cmd->joint_path.insert(
            planning_cmd->joint_path.begin(), q0);
    }

    checkSingularity(planning_cmd->joint_path);

    // 2. redistributing wpts
    if(redist_mode==REDIST_TYPE::NONE){
        return;
    }else if(redist_mode==REDIST_TYPE::UNIFORMWEIGHT){
        topt_traj_->redistQwptsPureNormDist(
            planning_cmd->joint_path,
            planning_cmd->joint_path);
    }else if(redist_mode==REDIST_TYPE::UNIFORMNORM){
        topt_traj_->redistQwptsNormDist(
            planning_cmd->joint_path,
            planning_cmd->joint_path);
    }else if(redist_mode==REDIST_TYPE::UNIFORM){
        // redistribute based on joint_arc / joint_max_vel
        // only consider velimit    
        topt_traj_->redistQwpts1st(planning_cmd->joint_path, 
                                vel_limit_,
                                planning_cmd->joint_path);
    } else if(redist_mode==REDIST_TYPE::TOPPRA){
        // redistribute based on toppra result:
        // it considers vel/acc/trq limit
        updateSysData(planning_cmd);    
        topt_solver_->solve(sys_data_,
                    grip_data_,
                    topt_traj_, // set topt_traj_ for redist
                    false); // not solve third order topp
        topt_traj_->redistQwpts2nd(planning_cmd->joint_path); 
    }          
}

void ToptPlanner::checkSingularity(std::vector<Eigen::VectorXd>& qwpts){
    // if there's joint jump, pull out the point 
    Eigen::VectorXd dq = qwpts[0];
    Eigen::VectorXd dq_sum = Eigen::VectorXd::Zero(dq.size()) ;
    int nqwpts = qwpts.size();
    for(int i(0); i<nqwpts-1; ++i){
        dq = qwpts[i+1] - qwpts[i];
        dq_sum += dq.cwiseAbs();        
    }
    dq_sum = dq_sum/((double)(nqwpts-1));

    int k(1);
    std::vector<int> erase_idx;
    Eigen::MatrixXd Jac;
    for(int i(2); i<nqwpts-3; ++i){
        dq = (qwpts[i] - qwpts[i-k])/((double)k);
        robot_temp_->updateSystem(qwpts[i], dq);
        Jac = robot_temp_->getBodyNodeJacobian(link_idx_);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if( svd.singularValues().coeff(5) < 1e-3
            && rossy_utils::getMaxRatioValue(dq, dq_sum) > 5.){
            std::cout<<"@@@@@ singularval = " << svd.singularValues().transpose() <<std::endl;            
            erase_idx.push_back(i);
            k+=1;
        } else k=1;
    }

    if(erase_idx.size()>0){
        is_singularity_in_path_ = true;
        std::cout<<"@@@@@ singularity detected!!!! - " << std::endl;
        std::cout<<"@@@@@ replacing from " << std::endl;
        erase_idx.push_back(nqwpts);
        int idx_prev = erase_idx[0];
        int idx_s = erase_idx[0];
        Eigen::VectorXd q, q0, q1;
        for(auto &idx : erase_idx){            
            if (idx_prev +1 < idx)
            {
                // replace idx_s ~ idx_prev
                std::cout <<"<replaced: " << idx_s << "~" << idx_prev << ">, ";
                int n = idx_prev-idx_s+1;                
                q0 = qwpts[idx_s-1];
                q1 = qwpts[idx_prev+1];
                for(int i(0); i<n; i++){
                    double alpha = ((double)(i+1))/((double)(n+1));
                    q = (1.-alpha)*q0 + alpha*q1;
                    qwpts[idx_s+i] = q;
                }
                idx_s = idx;
            }    
            std::cout << idx << ", ";        
            idx_prev = idx;
        }
        std::cout<<std::endl;
    }    
}

bool ToptPlanner::getSingularityCheckResult(){
    return is_singularity_in_path_;
}

bool ToptPlanner::doPlanning(PLANNING_COMMAND* planning_cmd) {  
    // user_cmd: WPT_DATA()
    if(b_planned_== false){
        b_planned_ = true;
        b_planned_firstvisit_=false;
        is_singularity_in_path_=false;

        WPT_DATA qwpt; 
        // preProcessingCommand(planning_cmd, REDIST_TYPE::UNIFORMNORM);
        preProcessingCommand(planning_cmd, REDIST_TYPE::UNIFORMWEIGHT);

        // compute system data on waypoints
        updateSysData(planning_cmd);            

        bool soln_exist = topt_solver_->solve(sys_data_,
                                            grip_data_,
                                            topt_traj_);
        
        if(soln_exist){
            sys_data_.savedata();

            start_time_ = current_time_;
            planned_time_ = topt_traj_->getMotionPeriod();
            end_time_ = start_time_ + planned_time_;
            std::cout<<" start_time_ = "<<start_time_<<std::endl;
            std::cout<<" planned_time_ = "<<planned_time_<<std::endl;
            std::cout<<" end_time_ = "<<end_time_<<std::endl; 
        }        

        return soln_exist;
    }else{
        std::cout<<" Planned trajectory isn't over"<<std::endl;
        return false;
    }
}

bool ToptPlanner::getPlannedCommand(Eigen::VectorXd& q_cmd) {
    if(b_planned_)
    {
        if(!b_planned_firstvisit_){
            start_time_ = current_time_;
            b_planned_firstvisit_=true;
        }
        topt_traj_->getCommand(current_time_ - start_time_ , q_cmd);        
        return true; 
    }

    return false;
}

bool ToptPlanner::getPlannedCommand(Eigen::VectorXd& q_cmd,
                                    Eigen::VectorXd& qdot_cmd) {
    if(b_planned_)
    {
        if(!b_planned_firstvisit_){
            start_time_ = current_time_;
            b_planned_firstvisit_=true;
        }
        topt_traj_->getCommand(current_time_ - start_time_ , 
                               q_cmd,  qdot_cmd);   
        return true; 
    }

    return false;        
}

bool ToptPlanner::getPlannedCommand(Eigen::VectorXd& q_cmd,
                                    Eigen::VectorXd& qdot_cmd,
                                    Eigen::VectorXd& qddot_cmd) {
    if(b_planned_)
    {
        if(!b_planned_firstvisit_){
            start_time_ = current_time_;
            b_planned_firstvisit_=true;
        }
        topt_traj_->getCommand(current_time_ - start_time_ , 
                                q_cmd, qdot_cmd, qddot_cmd);
        rossy_utils::saveVector(q_cmd,"Aaron/q");
        rossy_utils::saveVector(qdot_cmd,"Aaron/qdot");
        rossy_utils::saveVector(qddot_cmd,"Aaron/qddot");
        return true; 
    }

    return false;        
}

// ------------------- utils for python interface
void ToptPlanner::updateSysData(PLANNING_COMMAND* planning_cmd){   

    int n = planning_cmd->joint_path.size();      
    sys_data_.resize(n);

    double s = 0.;
    double ds = 1./((double)(n-1)); 
    for(int i(0); i<n; ++i){
        sys_data_.s[i] = s; 
        s += ds;
    }
    topt_traj_->setS2QSpline(planning_cmd->joint_path);

    // store sys data along the path
    Eigen::VectorXd q, dq, ddq;
    Eigen::MatrixXd J;
    Eigen::VectorXd dJdq;
    Eigen::VectorXd grav {{0., 0., -9.8}};

    double aratio=1.;
    int acc_percentage_idx = std::ceil(
        ((double)n-1.)*planning_cmd->acc_percentage_path_ratio);
    int dec_percentage_idx = std::floor(
        ((double)n-1.)*(1.-planning_cmd->dec_percentage_path_ratio));

    // std::cout<<" percentage_idx = 0 < " 
    //         << acc_percentage_idx <<" < " << dec_percentage_idx 
    //         << " < " << n-1 << std::endl;

    for(int i(0); i<n; ++i){
        s = sys_data_.s[i];
        // q = planning_cmd->joint_path[i];
        q = topt_traj_->spline_s2q_.evaluate(s);
        dq = topt_traj_->spline_s2q_.evaluateFirstDerivative(s);
        ddq = topt_traj_->spline_s2q_.evaluateSecondDerivative(s);
        robot_temp_->updateSystem(q, dq);
        sys_data_.q[i] = q;
        sys_data_.dq[i] = dq;
        sys_data_.ddq[i] = ddq;
    
        // for the cartesian vel/acc constraint
        sys_data_.ee[i] = robot_temp_->
            getBodyNodeIsometry(link_idx_).translation();
        J = robot_temp_->getBodyNodeJacobian(link_idx_);
        // J'(q,q')q'
        dJdq = robot_temp_->getBodyNodeJacobianDotQDot(link_idx_); 
        sys_data_.ee_v[i] = J.bottomRows(3)*dq;
        sys_data_.ee_a[i] = J.bottomRows(3)*ddq + dJdq.bottomRows(3);     

        // for the calculation of grasp constraitns
        J = robot_temp_->getBodyNodeBodyJacobian(link_idx_);
        // J'(q,q')q'
        dJdq = robot_temp_->getBodyNodeBodyJacobianDotQDot(link_idx_); 
        
        // (w,v) =  J(q)dq sdot
        // (aw, av) = (J(q)ddq + dJdq) sdot2  + J(q)dq sddot
        //    (aw1, av1: 1st order) sdot2 + (aw2, av2: 2nd order ) sddot
        sys_data_.ee_w[i] = J.topRows(3)*dq;
        sys_data_.ee_aw1[i] = J.topRows(3)*ddq + dJdq.topRows(3);
        sys_data_.ee_aw2[i] = J.topRows(3)*dq;
        sys_data_.ee_av1[i] = J.bottomRows(3)*ddq + dJdq.bottomRows(3);
        sys_data_.ee_av2[i] = J.bottomRows(3)*dq;
        sys_data_.ee_grav[i] = robot_temp_->
                getBodyNodeIsometry(link_idx_).linear().transpose()*grav;

        // M(q(s))*(q''ds2+q'dds)+C(q, q')q'ds2+g(q(s))
        //  = m dds + b ds2 + g
        // m=Mq', b=M(q)*q''+C(q,q')q', g=g(q)
        sys_data_.m[i] = robot_temp_->getMassMatrix()*dq;
        sys_data_.b[i] = robot_temp_->getMassMatrix()*ddq 
                    + robot_temp_->getCoriolisMatrix()*dq;
        sys_data_.g[i] = robot_temp_->getGravity();
        sys_data_.tm[i] = robot_temp_->GetTorqueUpperLimits();

        if(i<acc_percentage_idx) 
            aratio = planning_cmd->acc_percentage;
        else if(i>dec_percentage_idx) 
            aratio = planning_cmd->dec_percentage;
        else
            aratio = 1.;

        sys_data_.am[i] = aratio * planning_cmd->max_joint_acceleration;
        sys_data_.jm[i] = planning_cmd->max_joint_jerk;

        // (q'2)(ds2) 
        // Eigen::VectorXd v = robot_temp_->GetVelocityLowerLimits();
        Eigen::VectorXd v  = planning_cmd->max_joint_speed;
        sys_data_.av[i] = dq.cwiseProduct(dq);
        sys_data_.vm2[i] = v.cwiseProduct(v);

        // linear vel/acc limits, -1 will be given if inactivated
        sys_data_.lvm[i] = planning_cmd->max_linear_speed;
        sys_data_.lam[i] = planning_cmd->max_linear_acceleration;
    }    
}

void ToptPlanner::solveIK(const Eigen::VectorXd &q0,
             const std::vector<Eigen::VectorXd> &wpts,
             std::vector<Eigen::VectorXd> &qwpts){
    robot_manager_->solveIK(q0,wpts,qwpts);
}

void ToptPlanner::solveFK(const Eigen::VectorXd &q,
                        VEC_DATA* p) {
    std::cout<<"solveFK"<<std::endl;
    robot_temp_->updateSystem(q, 0.0*q);
    p->data = Eigen::VectorXd::Zero(7);
    p->data.head(3) = robot_temp_->getBodyNodeIsometry(link_idx_).translation();
    Eigen::Quaternion<double> ori_act = Eigen::Quaternion<double>(
            robot_temp_->getBodyNodeIsometry(link_idx_).linear()); 
    p->data[3]=ori_act.w();
    p->data[4]=ori_act.x();
    p->data[5]=ori_act.y();
    p->data[6] = ori_act.z();    
}

void ToptPlanner::solveFK(const Eigen::VectorXd &q,
                        const Eigen::VectorXd &qdot,
                        Eigen::VectorXd &x,
                        Eigen::VectorXd &xdot) {
    robot_temp_->updateSystem(q, qdot);
    x = Eigen::VectorXd::Zero(7);
    x.head(3) = robot_temp_->getBodyNodeIsometry(link_idx_).translation();
    Eigen::Quaternion<double> ori_act = Eigen::Quaternion<double>(
            robot_temp_->getBodyNodeIsometry(link_idx_).linear()); 
    x[3]=ori_act.w();
    x[4]=ori_act.x();
    x[5]=ori_act.y();
    x[6] = ori_act.z();

    // xdot = Jqdot, (v, w)
    Eigen::MatrixXd Jac = robot_temp_->getBodyNodeJacobian(link_idx_);
    // Eigen::MatrixXd Jac = robot_temp_->getBodyNodeBodyJacobian(link_idx_);
    xdot = Eigen::VectorXd::Zero(6); //
    xdot.segment<3>(0) = Jac.bottomRows(3)*qdot;
    xdot.segment<3>(3) = Jac.topRows(3)*qdot;
}

void ToptPlanner::computeTorques(TRAJ_DATA *traj_data,
                std::vector<Eigen::VectorXd> &trq_data){
    trq_data.clear();
    int nwpts = traj_data->qdata.size();
    int dim = robot_temp_->getNumDofs();   

    double dt;
    Eigen::VectorXd q, qdot, qddot, trq;    
    qdot = Eigen::VectorXd::Zero(dim);
    for (int i(0); i<nwpts-1; ++i){
        dt = traj_data->tdata[i+1] - traj_data->tdata[i];
        qddot = (traj_data->dqdata[i+1] - traj_data->dqdata[i])/dt;
        qdot = traj_data->dqdata[i];
        q = traj_data->qdata[i];
        robot_temp_->updateSystem(q, qdot);
        trq = robot_temp_->getMassMatrix()*qddot
            + robot_temp_->getCoriolisMatrix()*qdot
            + robot_temp_->getGravity();
        trq_data.push_back(trq);
    }
    // compute the last torque based on 
    // the assumption that 
    qdot = traj_data->dqdata[nwpts-1];
    q = traj_data->qdata[nwpts-1];
    robot_temp_->updateSystem(q, qdot);
    trq = robot_temp_->getCoriolisMatrix()*qdot
        + robot_temp_->getGravity();
    robot_temp_->getGravity();
        trq_data.push_back(trq);

}

void ToptPlanner::computeMaxLoad(const GRIP_DATA &gripdata,
                                TRAJ_DATA *traj_data){
    computeUnitLoad(gripdata, traj_data);
    int nwpts = traj_data->qdata.size(); 
    // U; [5n X 6n] grip stability inequality matrix
    // G; [6 X 6n] suction pad configuration matrix [AdT_it]T
    // fs; [6n X 1] suction force expressed in suc frames


    // 1. WeightAdjustment
    // 0,2,4,6 or 1,3,5,7
    Eigen::MatrixXd W1= Eigen::MatrixXd::Zero(0,0);
    Eigen::MatrixXd W2= Eigen::MatrixXd::Zero(0,0);
    Eigen::VectorXd Wn_inv {{1., 1., 0.422}}; // {{1.0, 1.0, 2.3682}};    
    Eigen::VectorXd Wc_inv {{1.195, 1.195, 7.570}}; // {{0.8369, 0.8369, 0.1321}};
    Eigen::VectorXd Wnormal4 = Wn_inv.replicate(4,1);
    Eigen::VectorXd Wcomprd4 = Wc_inv.replicate(4,1);
    Eigen::VectorXd pr = Eigen::VectorXd::Zero(3);
    Eigen::MatrixXd Ai = Eigen::MatrixXd::Zero(6,12);
    Eigen::MatrixXd As = Eigen::MatrixXd::Zero(0,0);    
    for(int i(0);i<4;++i){
        double r = gripdata.rpads(i);
        double theta = 2.*M_PI*((double)i)/4.;
        pr << r*cos(theta), r*sin(theta), 0.;
        Ai.block(0,3*i,3,3) = rossy_utils::skew(pr);
        Ai.block(3,3*i,3,3) = Eigen::MatrixXd::Identity(3,3);
    }
    for(int i(0);i<gripdata.n;++i){        
        As = rossy_utils::dStack(As,Ai);
        if(i%2==0){
            W1 = rossy_utils::dStack(W1, Wnormal4.asDiagonal() );
            W2 = rossy_utils::dStack(W2, Wcomprd4.asDiagonal() );
        }else{
            W1 = rossy_utils::dStack(W1, Wcomprd4.asDiagonal() );
            W2 = rossy_utils::dStack(W2, Wnormal4.asDiagonal() );
        }        
    }
    Eigen::MatrixXd A = gripdata.G * As;
    Eigen::MatrixXd AWA1 = A*W1*A.transpose();
    Eigen::MatrixXd AWA2 = A*W2*A.transpose();

    Eigen::LLT<Eigen::MatrixXd> ldlt1, ldlt2;
    ldlt1.compute(AWA1);
    ldlt2.compute(AWA2);
    Eigen::VectorXd Fi, Ft, b, let;
    double max_load1 = 100.;
    double max_load2 = 100.;
    for (int i(0); i<nwpts-1; ++i){
        Ft = traj_data->fsucdata[i];       
        
        // U (m*Fi-Fs) < 0
        // gripdata.U     
        // gripdata.fs
        b = gripdata.U*gripdata.fs;

        Fi = As*W1*A.transpose() * (ldlt1.solve(Ft));
        let = gripdata.U*Fi;        
        for(int k(0); k<let.size(); k++){
            if(let[k] > 0)
                max_load1 = std::min(max_load1, b[k]/let[k]);
        }
        Fi = As*W2*A.transpose() * (ldlt2.solve(Ft));
        let = gripdata.U*Fi;
        for(int k(0); k<let.size(); k++){
            if(let[k] > 0)
                max_load2 = std::min(max_load2, b[k]/let[k]);
        }
    }  
    std::cout<<" motion duration = " << traj_data->tdata[nwpts-1] << std::endl;
    std::cout<<" WeightAdjustment max_load = "<< max_load1 <<", " << max_load2 << std::endl;

    // 2. DDMethod : double description convex hull
    Polyhedron poly_1, poly_2;
    // 1. face(U)->span(V): U (mFi - Fs) < 0 -> mFi - fs = Vz
    bool b_poly_U = poly_1.setHrep(gripdata.U , Eigen::VectorXd::Zero(gripdata.U.rows()));
    Eigen::MatrixXd V = (poly_1.vrep().first).transpose(); // 6n x r

    // 2. span(G * V)->face(U_G): G(mFi) = mFt; G(Vz+fs) = mFt; GVz = mFt - fs -> U_G(mFt-fs) < 0
    Eigen::MatrixXd V_G = gripdata.G * V; // 6xr
    bool b_poly_V = poly_2.setVrep( V_G.transpose(), Eigen::VectorXd::Zero(V_G.cols()) );
    Eigen::MatrixXd U_G = poly_2.hrep().first; // px6

    // rossy_utils::pretty_print(V,std::cout,"V");
    // rossy_utils::pretty_print(V_G,std::cout,"V_G");
    rossy_utils::pretty_print(U_G,std::cout,"U_G");
    Eigen::VectorXd Gfs = gripdata.G*gripdata.fs;
    rossy_utils::pretty_print(Gfs,std::cout,"Gfs");
    b = U_G*gripdata.G*gripdata.fs;
    rossy_utils::pretty_print(b,std::cout,"b");


    //  3. m*U_G*Ft < U_G*G*fs -> m * (U_G*Ft) < b
    b = U_G*gripdata.G*gripdata.fs; // px1
    // let; // left eq term : px1
    double max_load = 100.;
    for (int i(0); i<nwpts-1; ++i){
        let = U_G*traj_data->fsucdata[i];
        for(int k(0); k<let.size()-3; k++){
            if(let[k] > 0.0)
                max_load = std::min(max_load, b[k]/let[k]);
        }
    }
    std::cout<<" dd method max_load = "<< max_load << std::endl;
}

void ToptPlanner::computeUnitLoad(const GRIP_DATA &gripdata,
                                    TRAJ_DATA *traj_data){
    // update traj_data->fsucdata
    // assume m = 1
    // EoM : unit F = [   I1 alpha  +  w x I1 w   ] 
    //                [   at − [pbox]×alpha - g   ] 
    
    int nwpts = traj_data->qdata.size();
    int dim = robot_temp_->getNumDofs();
    traj_data->fsucdata.resize(nwpts);
    Eigen::MatrixXd px = rossy_utils::skew(gripdata.p);
    Eigen::VectorXd grav {{0., 0., -9.8}};
    Eigen::VectorXd gt;    

    double dt;
    Eigen::VectorXd q, qdot, qddot;
    Eigen::VectorXd w, alpha, a;
    Eigen::MatrixXd J;
    Eigen::VectorXd dJdq;

    Eigen::VectorXd Ft = Eigen::VectorXd::Zero(6);
    for (int i(0); i<nwpts-1; ++i){
        // dt = 0.01;
        dt = traj_data->tdata[i+1] - traj_data->tdata[i];
        qddot = (traj_data->dqdata[i+1] - traj_data->dqdata[i])/dt;
        qdot = traj_data->dqdata[i];
        q = traj_data->qdata[i];
        robot_temp_->updateSystem(q, qdot);
        
        J = robot_temp_->getBodyNodeBodyJacobian(link_idx_);
        dJdq = robot_temp_->getBodyNodeBodyJacobianDotQDot(link_idx_); 
        w = J.topRows(3)*qdot;
        alpha = J.topRows(3)*qddot + dJdq.topRows(3);
        a = J.bottomRows(3)*qddot + dJdq.bottomRows(3);

        gt =  robot_temp_->
                getBodyNodeIsometry(link_idx_).linear().transpose()*grav;

        Ft.head(3) = gripdata.I1*alpha + rossy_utils::skew(w)*gripdata.I1*w;
        Ft.tail(3) = (a - px*alpha - gt );
        traj_data->fsucdata[i] = Ft;
    }

    // replicate the last value
    traj_data->fsucdata[nwpts-1] = traj_data->fsucdata[nwpts-2];
}

void ToptPlanner::computeSuctionForces(const GRIP_DATA &gripdata,
                TRAJ_DATA *traj_data,                
                std::vector<Eigen::VectorXd> &frc_data){

    frc_data.clear();
    int nwpts = traj_data->qdata.size();
    int dim = robot_temp_->getNumDofs();

    // EoM : F = [   I alpha  +  w x Iw   ] 
    //           [m(at − [pbox]×alpha - g)]  = G*Fi


    Eigen::MatrixXd px = rossy_utils::skew(gripdata.p);
    Eigen::VectorXd grav {{0., 0., -9.8}};
    Eigen::VectorXd gt;    
    Eigen::MatrixXd T_ot = rossy_utils::InverseSE3(
            Eigen::MatrixXd::Identity(3,3), gripdata.p);
    Eigen::MatrixXd AdT_to_trans_inverse = (rossy_utils::Ad_T(T_ot)).transpose();
    Eigen::VectorXd weg = Eigen::VectorXd::Zero(6*gripdata.n);
    for(int i(0);i<gripdata.n;++i){
        // Eigen::VectorXd w1 {{1.,1.,1.,1000.,1000.,1000.}};  
        // w.segment(i*w1.size(),w1.size()) = w1; 

        // assume suction force applied to the center of the half of the cup            
        double mu = gripdata.mus(i);
        double r = gripdata.rpads(i);
        double fmr = 1./(0.5*gripdata.rpads(i)); //force moment ratio:
        // weg.segment(i*6,6) << 1., 1., 1., fmr*fmr/(mu*mu), fmr*fmr/(mu*mu), fmr*fmr;        
        // weg.segment(i*6,6) << 1./(2.*r*r), 1./(2.*r*r), 1./(4.*r*r*mu*mu),  1./(2.*mu*mu), 1./(2.*mu*mu), 4.;
        // weg.segment(i*6,6) << 1., 1., 1., 1., 1., 1.;
        weg.segment(i*6,6) << r, r, r, 1., 1., 1.;
    }
    Eigen::MatrixXd WG = weg.asDiagonal() * gripdata.G.transpose();
    Eigen::MatrixXd GWG = gripdata.G * weg.asDiagonal() * gripdata.G.transpose();
    Eigen::LLT<Eigen::MatrixXd> ldlt;
    ldlt.compute(GWG);

    double dt;
    Eigen::VectorXd q, qdot, qddot;
    Eigen::VectorXd w, alpha, a;
    Eigen::MatrixXd J;
    Eigen::VectorXd dJdq;

    Eigen::VectorXd Ft = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd Fi = Eigen::VectorXd::Zero(6+6*gripdata.n);
    for (int i(0); i<nwpts-1; ++i){
        dt = 0.01;
        qddot = (traj_data->dqdata[i+1] - traj_data->dqdata[i])/dt;
        qdot = traj_data->dqdata[i];
        q = traj_data->qdata[i];
        robot_temp_->updateSystem(q, qdot);
        
        J = robot_temp_->getBodyNodeBodyJacobian(link_idx_);
        dJdq = robot_temp_->getBodyNodeBodyJacobianDotQDot(link_idx_); 
        w = J.topRows(3)*qdot;
        alpha = J.topRows(3)*qddot + dJdq.topRows(3);
        a = J.bottomRows(3)*qddot + dJdq.bottomRows(3);

        gt =  robot_temp_->
                getBodyNodeIsometry(link_idx_).linear().transpose()*grav;

        Ft.head(3) = gripdata.I*alpha + rossy_utils::skew(w)*gripdata.I*w;

        Ft.tail(3) = gripdata.m*(a - px*alpha - gt );
        Ft = AdT_to_trans_inverse*Ft;
        Fi = WG * (ldlt.solve(Ft));

        // rossy_utils::pretty_print(Fi, std::cout, "Fi");
        frc_data.push_back(rossy_utils::vStack(Ft, Fi));
        
    }
}