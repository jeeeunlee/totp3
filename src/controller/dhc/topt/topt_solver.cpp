#include "controller/dhc/topt/topt_solver.hpp"
#include "controller/dhc/topt/trajectory_manager.hpp"
#include "controller/dhc/topt/topt_utils.hpp"
#include "rossy_utils/math/math_utilities.hpp"
#include "rossy_utils/math/liegroup_utilities.hpp"
#include "clp/clpwrapper.hpp"

#include "rossy_utils/general/clock.hpp"

ToptSolver::ToptSolver(int dim) {    
    rossy_utils::pretty_constructor(2, "ToptSolver");
    dim_=dim; // robot actuator dimension
    lpsolver_ = new LPSolver();  
    grasping_activated_ = false;
    lin_vel_check_activated_ = false;
    lin_acc_check_activated_ = false;

    clock_ = new Clock();
    initializeDimensions();
}

void ToptSolver::initializeDimensions(){
    //
    lin_vel_check_activated_ = false;
    lin_acc_check_activated_ = false;
    if(sysdata_.getsize()>0){
        if( sysdata_.lvm[0] >0.)
            lin_vel_check_activated_ = true;
        if( sysdata_.lam[0] >0.)
            lin_acc_check_activated_ = true;
    }

    // 
    dimIneq1_ = dim_+1;
    if(lin_vel_check_activated_)
        dimIneq1_ += 3;
    //
    dimIneq2_ = 8*dim_;
    if(lin_acc_check_activated_)
        dimIneq2_+= 12;
    if(grasping_activated_)
        dimIneq2_+= (gripdata_.Ut.rows() + gripdata_.U.rows());
    //
    // dimIneq3_ = 2*dim_+1; // if spline condition added
    dimIneq3_ = 2*dim_+2; // if acc jump limit added
}

void ToptSolver::get1stlimit(int k, InequalData& constraints){
    //  A*x[k] < b
    // -1*x[k] < -MIN_SDOT_
    // dimIneq1_ = dim_ +1
    Eigen::VectorXd A,b;
    A = rossy_utils::vStack(sysdata_.av[k], -1.0);
    b = rossy_utils::vStack(sysdata_.vm2[k], -MIN_SDOT_);

    if(lin_vel_check_activated_){        
        double lv2max = sysdata_.lvm[k]*sysdata_.lvm[k];
        Eigen::VectorXd lv2 = sysdata_.ee_v[k].cwiseProduct(sysdata_.ee_v[k]);
        Eigen::VectorXd lv2max3 = Eigen::VectorXd::Constant(3, lv2max);        

        A = rossy_utils::vStack(A, lv2);
        b = rossy_utils::vStack(b, lv2max3);
    }
    constraints.A = A;
    constraints.b = b;
}

void ToptSolver::get2ndlimit(int k, InequalData& constraints){
    // A*[ x[k]; x[k+1] ] < b
    // dimIneq2_ = 4*dim_(trq) + 4*dim_(acc)
    constraints.A = Eigen::MatrixXd::Zero(8*dim_, 2);
    constraints.b = Eigen::VectorXd::Zero(8*dim_);

    // torque limit
    // 1) - tm[k] - g[k] <  
    //   m[k]/2ds[k]*x[k+1] + (b[k]-m[k]/2ds[k])*x[k] 
    //                                 < tm[k] - g[k]    
    Eigen::VectorXd ak1 = sysdata_.m[k]/2./(sysdata_.s[k+1]-sysdata_.s[k]);
    Eigen::VectorXd ak0 = sysdata_.b[k] - ak1;    
    constraints.A.block(0,0,dim_,1) = ak0;
    constraints.A.block(0,1,dim_,1) = ak1;    
    constraints.b.segment(0, dim_) = sysdata_.tm[k] - sysdata_.g[k];
    constraints.A.block(dim_,0,dim_,1) = -ak0;
    constraints.A.block(dim_,1,dim_,1) = -ak1;
    constraints.b.segment(dim_, dim_) = sysdata_.tm[k] + sysdata_.g[k];
    // 2) - tm[k+1] - g[k+1] <  
    //   (b[k+1]+m[k+1]/2ds[k])*x[k+1] -m[k+1]/2ds[k]*x[k] 
    //                                 < tm[k+1] - g[k+1]
    ak0 = - sysdata_.m[k+1]/2./(sysdata_.s[k+1]-sysdata_.s[k]);    
    ak1 = sysdata_.b[k+1] - ak0;
    constraints.A.block(2*dim_,0,dim_,1) = ak0;
    constraints.A.block(2*dim_,1,dim_,1) = ak1;    
    constraints.b.segment(2*dim_, dim_) = sysdata_.tm[k+1] - sysdata_.g[k+1];
    constraints.A.block(3*dim_,0,dim_,1) = -ak0;
    constraints.A.block(3*dim_,1,dim_,1) = -ak1;
    constraints.b.segment(3*dim_, dim_) = sysdata_.tm[k+1] + sysdata_.g[k+1];

    // acceleration limit
    // 1) -am[k] < 
    //  dq[k]/2ds[k] x[k+1] + (ddq[k]-dq[k]/2ds) x[k] 
    //                                        < am[k]
    ak1 = sysdata_.dq[k]/2./(sysdata_.s[k+1]-sysdata_.s[k]);
    ak0 = sysdata_.ddq[k] - ak1;
    constraints.A.block(4*dim_,0,dim_,1) = ak0;
    constraints.A.block(4*dim_,1,dim_,1) = ak1;    
    constraints.b.segment(4*dim_, dim_) = sysdata_.am[k];
    constraints.A.block(5*dim_,0,dim_,1) = -ak0;
    constraints.A.block(5*dim_,1,dim_,1) = -ak1;
    constraints.b.segment(5*dim_, dim_) = sysdata_.am[k];
    // 2) -am[k+1] < 
    // ( ddq[k+1] + dq[k+1]/2ds[k] ) x[k+1] - (dq[k+1]/2ds) x[k] 
    //                                        < am[k+1]
    ak0 = - sysdata_.dq[k+1]/2./(sysdata_.s[k+1]-sysdata_.s[k]);    
    ak1 = sysdata_.ddq[k+1] - ak0;
    constraints.A.block(6*dim_,0,dim_,1) = ak0;
    constraints.A.block(6*dim_,1,dim_,1) = ak1;    
    constraints.b.segment(6*dim_, dim_) = sysdata_.am[k+1];
    constraints.A.block(7*dim_,0,dim_,1) = -ak0;
    constraints.A.block(7*dim_,1,dim_,1) = -ak1;
    constraints.b.segment(7*dim_, dim_) = sysdata_.am[k+1];

    // grasping limit
    if(grasping_activated_)
        addgrasp2ndlimit(k, constraints);

    if(lin_acc_check_activated_){
        // linear accleration limit        
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(12,2);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(12);
        
        // 1) a = ee_a[k]*x[k]  + ee_v[k]*(x[k+1]-x[k])/2ds
        // -lam[k] < ee_v[k]/2ds x[k+1] + (ee_a[k]-ee_v[k]/2ds) x[k] < lam[k]        
        ak1 = sysdata_.ee_v[k]/2./(sysdata_.s[k+1]-sysdata_.s[k]);
        ak0 = sysdata_.ee_a[k] - ak1;
        A.block(0,0,3,1)=ak0;
        A.block(0,1,3,1)=ak1;
        A.block(3,0,3,1)=-ak0;
        A.block(3,1,3,1)=-ak1; 
        b.head(6) = Eigen::VectorXd::Constant(6,sysdata_.lam[k]);
        // 2) a = ee_a[k]*x[k+1]  + ee_v[k]*(x[k+1]-x[k])/2ds
        // -lam[k+1] < (ee_a[k+1]+ee_v[k+1]/2ds )x[k+1] - ee_v[k+1]/2ds x[k] < lam[k+1]
        ak0 = -sysdata_.ee_v[k+1]/2./(sysdata_.s[k+1]-sysdata_.s[k]);
        ak1 = sysdata_.ee_a[k+1] - ak0;
        A.block(6,0,3,1)=ak0;
        A.block(6,1,3,1)=ak1;
        A.block(9,0,3,1)=-ak0;
        A.block(9,1,3,1)=-ak1;
        b.tail(6) = Eigen::VectorXd::Constant(6,sysdata_.lam[k+1]);

        constraints.A = rossy_utils::vStack(constraints.A, A);
        constraints.b = rossy_utils::vStack(constraints.b, b);
    }
}

void ToptSolver::updateGripdataToSysdata(){
    // update suction 2nd order constraints
    // Fi = m_suc s'' + b_suc s'2 + g_suc
    // a_g*dds + b_g*ds2 + c_g < 0    

    // pre compute configuration matrices in gripper
    Eigen::VectorXd w = Eigen::VectorXd::Zero(6*gripdata_.n);
    double r;
    for(int i(0);i<gripdata_.n;++i){
        // assume force distribution around the suction cup 
        r = 0.95*gripdata_.rpads(i); // 0.95 < 1: safety factor
        w.segment(i*6,6) << 0.5*r*r, 0.5*r*r, r*r, 1., 1., 1.;
    }
    // suction forces represented in tool frame  
    Eigen::VectorXd fts = gripdata_.G * gripdata_.fs; 
    Eigen::MatrixXd T_ot = rossy_utils::InverseSE3(
            Eigen::MatrixXd::Identity(3,3), gripdata_.p);
    Eigen::MatrixXd AdT_ot_tr = (rossy_utils::Ad_T(T_ot)).transpose();

    Eigen::MatrixXd WG = w.asDiagonal() * gripdata_.G.transpose();
    Eigen::MatrixXd GWG = gripdata_.G * w.asDiagonal() * gripdata_.G.transpose();
    Eigen::LLT<Eigen::MatrixXd> ldlt;
    ldlt.compute(GWG);

    int nwpts = sysdata_.q.size();
    // a_g s'' + b_g s'2 + c_g < 0
    sysdata_.a_g.resize(nwpts);
    sysdata_.b_g.resize(nwpts);
    sysdata_.c_g.resize(nwpts);
    // Fi = m_suc s'' + b_suc s'2 + g_suc
    sysdata_.m_suc.resize(nwpts);
    sysdata_.b_suc.resize(nwpts);
    sysdata_.g_suc.resize(nwpts);

    Eigen::VectorXd a,b,c, tmp1, tmp2;
    Eigen::MatrixXd px, wx;
    for(int k(0); k<nwpts; ++k){
        // EoM : F + mg  = [I alpha + w x Iw] tmp1
        //                 [       m a      ] tmp2
        // Find (a,b,c) s.t. F = (a * dds + b*ds2 + c)
        px = rossy_utils::skew(gripdata_.p);
        wx = rossy_utils::skew(sysdata_.ee_w[k]);
        
        tmp1 = gripdata_.I*(sysdata_.ee_aw2[k]);
        tmp2 = gripdata_.m*(sysdata_.ee_av2[k] + px*sysdata_.ee_aw2[k]);
        a = rossy_utils::vStack(tmp1, tmp2);
        tmp1 = gripdata_.I*sysdata_.ee_aw1[k] + wx*gripdata_.I*sysdata_.ee_w[k];
        tmp2 = gripdata_.m*(sysdata_.ee_av1[k] + px*sysdata_.ee_aw1[k]);
        b = rossy_utils::vStack(tmp1, tmp2);
        tmp1 = Eigen::VectorXd::Zero(3);
        tmp2 = -gripdata_.m*sysdata_.ee_grav[k];
        c = rossy_utils::vStack(tmp1, tmp2);

        // Mapping the equations to tool frame        
        a = AdT_ot_tr*a;
        b = AdT_ot_tr*b;
        c = AdT_ot_tr*c;

        // Ft = G*Fi = {a * dds + b*ds2 + c}
        // Fi = WG'(GWG')^-1 {a * dds + b*ds2 + c}
        //    = m_suc dds + b_suc ds2 + g_suc        
        sysdata_.m_suc[k] = WG * (ldlt.solve(a));
        sysdata_.b_suc[k] = WG * (ldlt.solve(b));
        sysdata_.g_suc[k] = WG * (ldlt.solve(c));
        // Note: a,b,c can be retrived from G*(m_suc, b_suc, g_suc)


        // Ft = {a * dds + b*ds2 + c}
        // Ut*(Ft-fts) < 0
        // at*dds + bt*ds2 + ct < 0
        Eigen::VectorXd at, bt, ct;
        at = gripdata_.Ut*a;
        bt = gripdata_.Ut*b;
        ct = gripdata_.Ut*(c - fts);

        // U*(Fi-fs) < 0  
        // U*(m_suc dds + b_suc ds2 + g_suc -fs) < 0
        // ai * dds + bi*ds2 + ci < 0
        Eigen::VectorXd ai, bi, ci;
        ai = gripdata_.U * sysdata_.m_suc[k];
        bi = gripdata_.U * sysdata_.b_suc[k];
        ci = gripdata_.U * (sysdata_.g_suc[k] - gripdata_.fs);
        sysdata_.a_g[k] = rossy_utils::vStack(at, ai);
        sysdata_.b_g[k] = rossy_utils::vStack(bt, bi);
        sysdata_.c_g[k] = rossy_utils::vStack(ct, ci);
    }
}

void ToptSolver::addgrasp2ndlimit(int k, InequalData& constraints){
    
    // std::cout<<"grasping constraints added"<<std::endl;
    // a * dds + b*ds2 + c < 0

    // ak (x[k+1]-x[k])/(2 ds) + bk x[k] + ck < 0
    //    (bk-ak/2ds) * xk0 +  (ak/2ds) * xk1 < - ck
    Eigen::VectorXd ak0, ak1;
    ak0 = sysdata_.b_g[k]-sysdata_.a_g[k]/2./(sysdata_.s[k+1]-sysdata_.s[k]);
    ak1 = sysdata_.a_g[k]/2./(sysdata_.s[k+1]-sysdata_.s[k]);
    Eigen::MatrixXd A = rossy_utils::hStack(ak0, ak1);
    constraints.A = rossy_utils::vStack(constraints.A, A);
    constraints.b = rossy_utils::vStack(constraints.b, -sysdata_.c_g[k]);
    
}

void ToptSolver::get3rdlimit(int k, 
                            const std::vector<double>& x0_list,
                            InequalData& constraints ){ 
    // dddq[k] = (ddq[k+1] - ddq[k]) / dt[k]
    // ddq[k+1] - ddq[k] = J0 x[k] + J1 x[k+1] + J2 x[k+2]
    Eigen::VectorXd  J0, J1, J2;
    double dsk = sysdata_.s[k+1]-sysdata_.s[k];
    double dskk = sysdata_.s[k+2]-sysdata_.s[k+1];
    // std::cout<<"dsk="<<dsk<<", dskk="<<dskk<<std::endl;
    J0 = sysdata_.dq[k]/2./dsk - sysdata_.ddq[k];
    J1 = -sysdata_.dq[k+1]/2./dskk - sysdata_.dq[k]/2./dsk + sysdata_.ddq[k+1];
    J2 = sysdata_.dq[k+1]/2./dskk;
    // std::cout<<"J0="<<J0.transpose()<<", J1="<<J1.transpose()
    //             <<", J2="<<J2.transpose()<<std::endl;

    // dt[k] = h(xbar, k) > hbar + dh0*x[k] + dh1*x[k+1] + dh2*x[k+2]  
    double vk0, vk1, vk2;
    vk0 = sqrt(std::max(MIN_SDOT_, x0_list[k]));
    vk1 = sqrt(std::max(MIN_SDOT_, x0_list[k+1]));
    vk2 = sqrt(std::max(MIN_SDOT_, x0_list[k+2]));

    // need to check if vk0, vk1, vk2 are zero or not
    double hbar, dh0, dh1, dh2;
    if(k==0){
        dh0 = 0.;
        dh1 = (- dsk/(vk0+vk1)/(vk0+vk1) - dskk/(vk1+vk2)/(vk1+vk2)) * 0.5/vk1 ;
        dh2 = - dskk/(vk1+vk2)/(vk1+vk2) * 0.5/vk2;
    }else if(k==x0_list.size()-3){
        dh0 = - dsk/(vk0+vk1)/(vk0+vk1) * 0.5/vk0;
        dh1 = - (dsk/(vk0+vk1)/(vk0+vk1) 
                    + dskk/(vk1+vk2)/(vk1+vk2)) * 0.5/vk1 ;
        dh2 = 0.;
    }else{
        dh0 = - dsk/(vk0+vk1)/(vk0+vk1) * 0.5/vk0;
        dh1 = - ( dsk/(vk0+vk1)/(vk0+vk1) 
                    + dskk/(vk1+vk2)/(vk1+vk2) ) * 0.5/vk1 ;
        dh2 = - dskk/(vk1+vk2)/(vk1+vk2) * 0.5/vk2;
    }
    hbar = dsk/(vk0 + vk1) + dskk/(vk1 + vk2);
    // std::cout<<"hbar= "<< hbar <<", dh0="<<dh0<<", dh1="<<dh1
    //             <<", dh2="<<dh2<<std::endl;
    hbar = hbar-dh0*x0_list[k] -dh1*x0_list[k+1] -dh2*x0_list[k+2];

    // add jerk condition   
    constraints.A.resize(2*J0.size(), 3);
    constraints.A.col(0) << -J0 - sysdata_.jm[k]*dh0, J0 - sysdata_.jm[k]*dh0;
    constraints.A.col(1) << -J1 - sysdata_.jm[k]*dh1, J1 - sysdata_.jm[k]*dh1;
    constraints.A.col(2) << -J2 - sysdata_.jm[k]*dh2, J2 - sysdata_.jm[k]*dh2;

    // std::cout<<constraints.A<<std::endl;

    Eigen::VectorXd temp;
    temp = sysdata_.jm[k]*hbar;
    constraints.b = rossy_utils::vStack( temp , temp );

    // std::cout<<constraints.b<<std::endl;

    // add spline condition
    // (-1/3-2r)x[k]+(-5/3-2/3*dsk/dskk-2(1-r))x[k+1]+2/3*(dsk/dskk)x[k+2] < 0  
    // Eigen::MatrixXd Asp = Eigen::MatrixXd::Zero(1,3);
    //  Asp << (-1./3.), (-5./3.-2./3.*dsk/dskk), 2./3.*dsk/dskk;
    // constraints.A = rossy_utils::vStack(constraints.A, Asp);
    // constraints.b = rossy_utils::vStack(constraints.b, 0.);

    // add 
    // -J < q'[k+1]/tstep * ( (x[k+2]-x[k+1])/dskk  - (x[k+1]-x[k])/dsk ) < J
    // -J/q'[k+1]*tstep < (1/dskk)*x[k+2] - (1/dskk+1/dsk)*x[k+1] + (1/dsk)*x[k]
    Eigen::MatrixXd Asp = Eigen::MatrixXd::Zero(2,3);
    Asp << -1./dsk,  (1./dskk+1./dsk), -(1./dskk),
            1./dsk, - (1./dskk+1./dsk), (1./dskk);
    double dqdj = rossy_utils::getMaxRatioValue( sysdata_.dq[k+1], sysdata_.jm[k]);
    double tstep = 0.02;
    Eigen::VectorXd bsp {{ tstep/dqdj, tstep/dqdj}};
    constraints.A = rossy_utils::vStack(constraints.A, Asp);
    constraints.b = rossy_utils::vStack(constraints.b, bsp);

}

double ToptSolver::getFeasibleX(int k) {
    // given x[k+1] 
    // min -x[k]
    // s.t. ax*x[k] < b
    Eigen::VectorXd ax, b;
    InequalData constraints;

    // add velocity constraints: A[0]*x[k] < b
    get1stlimit(k, constraints);    
    ax = constraints.A.col(0);
    b = constraints.b;

    // solve LP
    double xmax_c_k;    
    double ret = rossy_utils::linprog1d(-1, ax, b, xmax_c_k);
    return xmax_c_k;
}

double ToptSolver::getControllableX(int k, double xmax_c_kk){
    // given: x[k+1] < xmax_c_kk
    // find x[k], x[k+1], min t
    // s.t. A0*x[k] + A1*x[k+1] < b

    // INPUT: k, xmax_c_kk
    // OUTPUT: xmax_c_k,
    double xmax_c_k;
    
    // add velocity constraints: A[0]*x[k] < b
    InequalData cst1;
    get1stlimit(k, cst1);    

    // add torque limit constraints: A[0]*x[k] + A[1]*x[k+1] < b
    InequalData cst2;
    get2ndlimit(k, cst2);

    // case1 : if x_kk is fixed:
    if(xmax_c_kk < MIN_SDOT_ ){
        Eigen::VectorXd ax = cst1.A.col(0);
        ax = rossy_utils::vStack(ax, cst2.A.col(0) );
        Eigen::VectorXd b = cst1.b;
        b = rossy_utils::vStack(b, cst2.b - cst2.A.col(1)*xmax_c_kk );        
        double ret = rossy_utils::linprog1d(-1, ax, b, xmax_c_k);         
        return xmax_c_k;
    }

    // case2 : find x[k] that maximizes{ x[k] + x[k+1] } 
    Eigen::MatrixXd Ax;
    Eigen::VectorXd b;
    Eigen::VectorXd ax = cst1.A.col(0);
    int colsize = cst1.A.rows();
    // 1nd limit: A[0]*x[k] + 0*x[k+1] < b
    Ax = rossy_utils::hStack(ax, 
            Eigen::VectorXd::Zero(colsize));
    // 2nd limit: : A[0]*x[k] + A[1]*x[k+1] < b
    Ax = rossy_utils::vStack(Ax , cst2.A);
    b = rossy_utils::vStack(cst1.b, cst2.b);
    // bwd reachability:  x[k+1] < xmax_c_kk
    Eigen::MatrixXd Axtmp {{0., 1.}};
    Ax = rossy_utils::vStack(Ax, Axtmp);    
    b = rossy_utils::vStack(b, xmax_c_kk);
    // solve LP
    Eigen::VectorXd f2d {{-1, -1}};
    // Eigen::VectorXd f2d {{-1, 0}};
    Eigen::VectorXd soln;    
    double ret = rossy_utils::linprog2d(f2d, Ax, b, soln);

    // return soln[0];

    // if solution looks ok
    if(soln[0] > 2*MIN_SDOT_ && soln[1] > 2*MIN_SDOT_
        && soln[0]*10. > soln[1] && soln[1]*10. > soln[0] ){          
        xmax_c_k = soln[0];
        xmax_c_kk = soln[1];
        return xmax_c_k;
    }else{
        std::cout<<" HERE!!! solution not good @ "<< k << std::endl;
        std::cout << "soln = " << soln[0] << ", " << soln[1] << std::endl;
        int N=7;
        double xkk_tmp, xmaxk_tmp;
        double t(0.), mint(1000.);
        xmax_c_k = MIN_SDOT_;
        double given_xmax_c_kk = xmax_c_kk;
        for(int i(0); i<N; ++i){
            xkk_tmp = (double)i/(double)(N-1)*MIN_SDOT_ 
                    + (double)(N-1-i)/(double)(N-1)*given_xmax_c_kk;

            Eigen::VectorXd ax = cst1.A.col(0);
            ax = rossy_utils::vStack(ax, cst2.A.col(0) );
            Eigen::VectorXd b = cst1.b;
            b = rossy_utils::vStack(b, cst2.b - cst2.A.col(1)*xkk_tmp );            
            ret = rossy_utils::linprog1d(-1, ax, b, xmaxk_tmp);
            t = 1/sqrt(xkk_tmp) + 1/sqrt(xmaxk_tmp);

            if(t < mint){                
                mint = t;
                xmax_c_kk = xkk_tmp;
                xmax_c_k = xmaxk_tmp;
            }
        }
        std::cout << "soln = " << xmax_c_k << ", " << xmax_c_kk << std::endl;
        return xmax_c_k;
    }    
}

double ToptSolver::getReachableXMax(int k, double xmax_c_k, double xmax_r_kpre){
    // given x[k-1] 
    // min -x[k]
    // s.t. ax1*x[k] < b-ax0*x[k-1]
    Eigen::VectorXd ax, b;
    InequalData constraints;

    // add next state controllable
    get2ndlimit(k-1, constraints);
    ax = constraints.A.col(1);
    b = constraints.b - constraints.A.col(0)*xmax_r_kpre;

    // add next state controllable
    ax = rossy_utils::vStack(ax, 1);
    b = rossy_utils::vStack(b, xmax_c_k);

    ax = rossy_utils::vStack(ax, -1);
    b = rossy_utils::vStack(b, -MIN_SDOT_);

    double xmax_r_k, ret;
    ret = rossy_utils::linprog1d(-1, ax, b, xmax_r_k);
    return xmax_r_k;
}

bool ToptSolver::solve(const SYSTEM_DATA &sysdata,
                        const GRIP_DATA &gripdata,
                        TrajectoryManager* traj,
                        bool thirdorder){
    // set system variables
    sysdata_ = sysdata;
    gripdata_ = gripdata;
    grasping_activated_ = true; 
    if(grasping_activated_)
        updateGripdataToSysdata();

    // 0. initialize sets containers
    initializeSets();
    initializeDimensions();

    // 1. compute controllable_xmax_, reachable_xmax_    
    clock_->start();
    bool soln_exist = solveTOPPRA();
    std::cout<<"DHC: solveTOPPRA()="<<clock_->stop()<<"ms"<<std::endl;    
        
    // return if no jerk limit violation 
    // std::cout<<"DHC: ------ toppra - 2nd order ------ " << std::endl;
    // double limitratio = checkLimits(reachable_xmax_);
    // checkForcesAtSuction(reachable_xmax_);
    double limitratio = 2.0;

    if( !soln_exist ){
        std::cout<<"DHC: ------ toppra - 2nd order ------ " << std::endl;
        limitratio = checkLimits(reachable_xmax_);
        std::cout<<"DHC: solveTOPPRA() -> no solution exists"<<std::endl;
        return false;
    }  
    if( !thirdorder || limitratio < 1.0 + 1e-5 ){        
        // traj->setT2QSpline(sysdata_.s, reachable_xmax_);
        traj->setT2SSpline(sysdata_.s, reachable_xmax_);
        return true;
    }

    // 2. compute trackable_xmax_
    clock_->start();
    solveTOPP3(reachable_xmax_);
    std::cout<<"DHC: solveTOPP3()="<<clock_->stop()<<"ms"<<std::endl;

    // 3. generate spline_t2s_
    clock_->start();
    // traj->setT2QSpline(sysdata_.s, trackable_xmax_);
    traj->setT2SSpline(sysdata_.s, trackable_xmax_);
    std::cout<<"DHC: setT2QSpline()="<<clock_->stop()<<"ms"<<std::endl;    

    // Check Results
    std::vector<double> periods;
    traj->getMotionPeriods(periods);
    
    // check
    std::cout<<"DHC: ------ toppra - 3rd order ------ " << std::endl;
    checkLimits(trackable_xmax_);
    checkForcesAtSuction(trackable_xmax_);

    return true;
}

void ToptSolver::resolve(double planning_time, 
                        const GRIP_DATA &gripdata,
                        TrajectoryManager* traj){
    gripdata_ = gripdata;
    initializeDimensions();    

    // 1. Find the next interval
    int idx = traj->evaluateTimeInterval(planning_time);
    int nwpts = sysdata_.getsize();
    std::cout << idx <<"- th interval / " << nwpts << std::endl;

    if( idx+1 < nwpts ) idx++;
    else return;

    // 2. solve TOPPRA
    clock_->start();
    controllable_xmax_ = trackable_xmax_;
    reachable_xmax_ = trackable_xmax_;
    solveTOPPRA(idx);
    std::cout<<"solveTOPPRA()="<<clock_->stop()<<"ms"<<std::endl; 

    // 3. solve TOPP3
    clock_->start();
    solveTOPP3(reachable_xmax_, idx);
    std::cout<<"solveTOPP3()="<<clock_->stop()<<"ms"<<std::endl;  

    // 4. reset spline
    clock_->start();
    traj->setT2QSpline(sysdata_.s, trackable_xmax_);
    std::cout<<"setT2QSpline()="<<clock_->stop()<<"ms"<<std::endl;    
}   


void ToptSolver::initializeSets(){
    int num_wpts = sysdata_.q.size();   

    controllable_xmax_.resize(num_wpts);
    reachable_xmax_.resize(num_wpts); 
    trackable_xmax_.resize(num_wpts); 
    std::fill(controllable_xmax_.begin(), controllable_xmax_.end(), 0.);
    std::fill(reachable_xmax_.begin(), reachable_xmax_.end(), 0.);
    std::fill(trackable_xmax_.begin(), trackable_xmax_.end(), 0.);

}

void ToptSolver::solveTOPPRA0(){
    // only consider 1st order constraints
    int num_wpts = sysdata_.q.size(); 
    std::cout<<"num_wpts="<<num_wpts<<std::endl;
    // 
    feasible_xmax_.resize(num_wpts);
    feasible_xmax_[num_wpts-1] = 0.;
    feasible_xmax_[0] = 0.;
    // 
    double xmax;
    std::cout<<"feasible_xmax_ (inverse order) = ";
    for(int i(num_wpts-2); i>0; --i){
        xmax = getFeasibleX(i);
        feasible_xmax_[i] = xmax;
        std::cout<<xmax<<", ";
    }
    std::cout<<std::endl;
}

bool ToptSolver::solveTOPPRA(int i_c){
    // solve for x[i_c+1]~x[num_wpts-2]
    // x[0],..., x[i_c], and x[num_wpts-1] are fixed    
    int num_wpts = sysdata_.q.size();       

    // backward 
    // update controllable_xmax_ for i>i_c
    double xmax_c = 0.;
    for(int i(num_wpts-2); i>i_c-1; --i){
        xmax_c = getControllableX(i, xmax_c);
        controllable_xmax_[i] = xmax_c;
    }
    // rossy_utils::pretty_print(controllable_xmax_, "controllable_xmax_");    

    // forward    
    // update controllable_xmax_ for i>i_c  
    double xmax_r = reachable_xmax_[i_c];
    for(int i(i_c+1); i<num_wpts; ++i){
        xmax_c = controllable_xmax_[i];
        xmax_r = getReachableXMax(i, xmax_c, xmax_r);
        reachable_xmax_[i] = xmax_r;
    }
    // rossy_utils::pretty_print(reachable_xmax_, "reachable_xmax_"); 

    // check if solution exists
    // if x = 0 during the motion
    for(int i(i_c+1); i<num_wpts-1; ++i){
        if( reachable_xmax_[i] < 0.5*MIN_SDOT_ )
            return false;
    }
    return true;

}

void ToptSolver::solveTOPP3(
        const std::vector<double>& x0_list, int i_c) {    
    // jerk consider   
    int num_wpts = sysdata_.q.size();
    std::cout <<" solveTOPP3 : num_wpts=" << num_wpts << std::endl;
    
    // x[0],..., x[i_c], and x[num_wpts-1] are fixed
    int nh = num_wpts-2-i_c; // x[i_c+1]~x[num_wpts-2 = i_c+nh]    

    // initial iteration
    InequalDataList constraints;
    Eigen::VectorXd f, x;
    std::vector<double> xprev_list = x0_list;
    std::vector<double> xopt_list = x0_list;    
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(0,0);
    Eigen::VectorXd b = Eigen::VectorXd::Zero(0);
    double topt = estimateMotionPeriod(x0_list);
    double alpha = 10.; // trust region constant
    constraints = buildTOPP3Ineq(xprev_list, i_c, nh, alpha);
    for(auto & cst: constraints){
        A = rossy_utils::vStack(A, cst.A);
        b = rossy_utils::vStack(b, cst.b);
    }
    // f = Eigen::VectorXd::Constant(nh, -1);
    f = getCostCoeffs(x0_list, i_c, nh);
    double ret = lpsolver_->solve(f, A, b, x);

    // update xnew
    double x_diff(0.), x_norm(0.), t_diff(1.);
    for(int k(1); k<nh+1; k++){     
        x_norm += x(k-1)*x(k-1);
        x_diff += (xprev_list[i_c+k]-x(k-1))*(xprev_list[i_c+k]-x(k-1));  
        xprev_list[i_c+k] = x(k-1);    
    }           
    x_diff = sqrt(x_diff/x_norm);
    double t = estimateMotionPeriod(xprev_list);    
    
    t_diff = fabs(topt-t);
    topt = t;
    xopt_list = xprev_list;
    alpha = 1.0;
    std::cout<<"x_diff = "<<x_diff<<", t="<<t<<", topt="<<topt<<std::endl;

    int iter=0;    
    while(++iter<10 && x_diff>1e-2 && t_diff>0.01){        
        // build A, b : x[i_c+1:i_c+nh]
        updateTOPP3Ineq(xprev_list, i_c, nh, constraints, alpha); 
        int n = constraints[2].b.size();
        A.bottomRows(n)  = constraints[2].A;
        b.tail(n) =  constraints[2].b;    
        f = getCostCoeffs(xprev_list, i_c, nh);
        double ret = lpsolver_->solve(f, A, b, x);
        
        // update xnew
        x_norm=0.;
        x_diff=0.;
        for(int k(1); k<nh+1; k++){ 
            x_norm += x(k-1)*x(k-1);         
            x_diff += (xprev_list[i_c+k]-x(k-1))*(xprev_list[i_c+k]-x(k-1));  
            xprev_list[i_c+k] = x(k-1);    
        }           
        x_diff = sqrt(x_diff/x_norm);
        t = estimateMotionPeriod(xprev_list);
        if(t < topt){
            alpha *= 1.5;
            t_diff = fabs(topt-t);
            topt = t;
            xopt_list = xprev_list;
        } else{
            alpha *= 0.5;
            xprev_list = xopt_list;
        }    
        std::cout<<"x_diff = "<<x_diff<<", t="<<t<<", topt="<<topt
                <<", alpha="<<alpha<<std::endl;        
    }
    trackable_xmax_ = xopt_list;            
}

Eigen::VectorXd ToptSolver::getCostCoeffs(
    const std::vector<double>& x0_list, int k, int h){    
    // min f'x, with opt vars : x=x[k+1]~x[k+h]
    Eigen::VectorXd f = Eigen::VectorXd::Constant(h, -1);

    double xk0, xk1, xk2, dsk0, dsk1;
    for(int i(0); i<h; ++i){
        xk0 = sqrt(std::max(0.0, x0_list[k+i]));
        xk1 = sqrt(std::max(MIN_SDOT_, x0_list[k+i+1]));
        xk2 = sqrt(std::max(0.0, x0_list[k+i+2]));

        dsk0 = sysdata_.s[k+i+2]-sysdata_.s[k+i+1];
        dsk1 = sysdata_.s[k+i+1]-sysdata_.s[k+i];

        f[i] = - ( dsk0/(xk0+xk1)/(xk0+xk1) 
                    + dsk1/(xk2+xk1)/(xk2+xk1) ) / xk1;
    }
    return f;
}

void ToptSolver::updateTOPP3Ineq(        
        const std::vector<double>& x0_list,
        int k, int h, InequalDataList& TOPP3Ineq, double alpha){
    // update 3rd order constraints for x[k+1]~x[k+h]
    InequalData cntrt_tmp; 

    // Assume x[k](& x[k-1]) are fixed
    // x[k+h+1] is fixed if it's the last point
    bool b_last_fixed = (k+h+1 == x0_list.size()-1);

    int i0 = 0;
    int dim3rdIneq = dimIneq3_*(h-1);       
    if(k>0) {dim3rdIneq+=dimIneq3_; i0=1;}
    if(b_last_fixed) dim3rdIneq+=dimIneq3_;

    Eigen::MatrixXd A3 = Eigen::MatrixXd::Zero(dim3rdIneq, h);
    Eigen::VectorXd b3 = Eigen::VectorXd::Zero(dim3rdIneq);
    
    double xk = x0_list[k];
    
    if(k>0){
        // x[k-1], x[k] fixed
        // A2*x[k+1] < b - A0*x[k-1] - A1*x[k]
        double xkpre = x0_list[k-1];
        get3rdlimit(k-1, x0_list, cntrt_tmp);
        A3.block(0, 0, dimIneq3_, 1) = cntrt_tmp.A.rightCols(1);
        b3.segment(0, dimIneq3_) = cntrt_tmp.b 
                            - cntrt_tmp.A.col(1)*xk 
                            - cntrt_tmp.A.col(0)*xkpre;
    }
    // A1*x[k+1] +A2*x[k+2] < b - A0*x[k]
    get3rdlimit(k, x0_list, cntrt_tmp);
    A3.block(i0*dimIneq3_, 0, dimIneq3_, 2) = cntrt_tmp.A.rightCols(2);
    b3.segment(i0*dimIneq3_, dimIneq3_) = cntrt_tmp.b 
                                        - cntrt_tmp.A.col(0)*xk;
    for(int i(1); i<h-1; i++){
        // A0*x[k+i] + A1*x[k+i+1] +A2*x[k+i+2] < b
        get3rdlimit(k+i, x0_list, cntrt_tmp);
        A3.block((i0+i)*dimIneq3_, i-1, dimIneq3_, 3) = cntrt_tmp.A;
        b3.segment((i0+i)*dimIneq3_, dimIneq3_) = cntrt_tmp.b;
    }
    if(b_last_fixed){
        // k+h+1 = N-1 : last element
        // A0*x[k+h-1] + A1*x[k+h] < b - A2*{x[k+h+1]=0}
        get3rdlimit(k+h-1, x0_list, cntrt_tmp);
        A3.bottomRightCorner(dimIneq3_, 2) = cntrt_tmp.A.leftCols(2);
        b3.tail(dimIneq3_) = cntrt_tmp.b;
    }

    // end condition
    if(k==0){ 
        // add jerk constraint with zero acc assumption at the begining
        // qddot[0] / 0.5*dt[0] < jm 
        // (dq[0]/2/ds/ds)*x[1]^3/2 < Jm
        double ds = sysdata_.s[1]-sysdata_.s[0];
        Eigen::VectorXd dq0 = sysdata_.dq[0]/2./ds/ds;
        double tmp = rossy_utils::getMaxRatioValue(dq0, sysdata_.jm[0]);
        // x[1] < (1/tmp)^2/3
        tmp = std::pow(1./tmp , 2./3.);
        Eigen::MatrixXd Atmp = Eigen::MatrixXd::Zero(1,h);
        Atmp(0,0)=1.;
        A3 = rossy_utils::vStack(A3, Atmp);
        b3 = rossy_utils::vStack(b3, tmp);

    }
    if(b_last_fixed){
        // k+h+1 = N-1 : last element
        // qddot[N-2] / 0.5*dt[N-2] < jm 
        // (ddq[N-2]-dq[N-2]/2/ds)/ds * x[N-2]^3/2 < jm
        double ds = sysdata_.s[k+h+1]-sysdata_.s[k+h];
        Eigen::VectorXd dq0 
            = (sysdata_.ddq[k+h]-sysdata_.dq[k+h]/2./ds)/ds;
        double tmp = rossy_utils::getMaxRatioValue(dq0, sysdata_.jm[k+h+1]);
        // x[N-2] < (1/tmp)^2/3
        tmp = std::pow(1./tmp , 2./3.);
        Eigen::MatrixXd Atmp = Eigen::MatrixXd::Zero(1,h);
        Atmp(0,h-1)=1.;
        A3 = rossy_utils::vStack(A3, Atmp);
        b3 = rossy_utils::vStack(b3, tmp);
    }
    //
    if(alpha>0.){
        // x - r < x[k+1]~x[k+h] < x + r
        Eigen::MatrixXd Atmp = Eigen::MatrixXd::Identity(h,h);
        Eigen::VectorXd btmp = Eigen::VectorXd::Zero(2*h);
        for(int i(0); i<h; i++){
            double r = alpha*x0_list[k+1+i];
            btmp(i) = x0_list[k+1+i]+r;
            btmp(h+i) = -x0_list[k+1+i]+r;
        }
        Atmp = rossy_utils::vStack(Atmp, -Atmp);
        A3 = rossy_utils::vStack(A3, Atmp);
        b3 = rossy_utils::vStack(b3, btmp);
    }

    TOPP3Ineq[2].A = A3;
    TOPP3Ineq[2].b = b3;

}

InequalDataList ToptSolver::buildTOPP3Ineq(
        const std::vector<double>& x0_list,
        int k, int h, double alpha){
    // Build constraitns for x[k+1:k+h]
    InequalDataList result;   
    InequalData cntrt_tmp; 

    // Assume x[k](& x[k-1]) are fixed
    // x[k+h+1] is fixed if it's the last point
    bool b_last_fixed = (k+h+1 == x0_list.size()-1);      

    // 1. 1st order constraints - vel limits
    Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero(dimIneq1_*h,h); 
    Eigen::VectorXd b1 = Eigen::VectorXd::Zero(dimIneq1_*h);    
    for(int i(0); i<h; ++i){
        get1stlimit(k+1+i, cntrt_tmp);
        A1.block(i*dimIneq1_, i, dimIneq1_,1) = cntrt_tmp.A.col(0);
        b1.segment(i*dimIneq1_,dimIneq1_) = cntrt_tmp.b;
    }
    result[0].A = A1;
    result[0].b = b1;
    
    // 2. 2nd order constraints - trq/acc limits
    int dim2ndIneq = dimIneq2_*h;
    if(b_last_fixed) dim2ndIneq+=dimIneq2_;
    Eigen::MatrixXd A2 = Eigen::MatrixXd::Zero(dim2ndIneq,h); 
    Eigen::VectorXd b2 = Eigen::VectorXd::Zero(dim2ndIneq);
   
    // A1*x[k+1] < b - A0*x[k]
    double xk = x0_list[k];
    get2ndlimit(k, cntrt_tmp);
    A2.block(0,0,dimIneq2_,1) = cntrt_tmp.A.rightCols(1);
    b2.segment(0, dimIneq2_) = cntrt_tmp.b - cntrt_tmp.A.col(0)*xk;
    for(int i(1); i<h; i++){
        // A0*x[k+i] + A1*x[k+i+1] < b 
        get2ndlimit(k+i, cntrt_tmp);
        A2.block(i*dimIneq2_, i-1, dimIneq2_, 2) = cntrt_tmp.A;
        b2.segment(i*dimIneq2_, dimIneq2_) = cntrt_tmp.b;
    }
    if(b_last_fixed){
        // A0*x[k+h] < b - A1*x[k+h+1] 
        get2ndlimit(k+h, cntrt_tmp);
        A2.block(h*dimIneq2_, h-1, dimIneq2_, 1) = cntrt_tmp.A.leftCols(1);
        b2.segment(h*dimIneq2_, dimIneq2_) = cntrt_tmp.b;
    }
    result[1].A = A2;
    result[1].b = b2;

    // 3. 3rd order - jerk    
    updateTOPP3Ineq(x0_list, k, h, result, alpha); // updating result[2]
    
    return result;
}

// check functions
void ToptSolver::checkForcesAtSuction(
        const std::vector<double>& x0_list){
    
    // /*--------------------------------------------------------*/
    int num_wpts = sysdata_.q.size();    
    double dsk, uk;
    Eigen::VectorXd fsuc_i = Eigen::VectorXd::Zero(42);
    // std::cout<<" checkForcesAtSuction "<< std::endl;
    // // suction force / torque
    // for(int k(0); k<num_wpts; ++k){
    //     if(k==num_wpts-1){
    //         dsk = sysdata_.s[k] - sysdata_.s[k-1];
    //         uk = 0.;
    //     }else{
    //         dsk = sysdata_.s[k+1] - sysdata_.s[k];
    //         uk = (x0_list[k+1] - x0_list[k]) / (2*dsk);                 
    //     }        
    //     fsuc_i = sysdata_.m_suc[k]*uk 
    //             + sysdata_.b_suc[k]*x0_list[k] 
    //             + sysdata_.g_suc[k];
    //     // force
    //     for(int j(0); j<gripdata_.n; j++)               
    //         std::printf("% 5.2f, % 5.2f, % 5.2f /", fsuc_i[6*j+3], fsuc_i[6*j+4], fsuc_i[6*j+5]);
    //     std::cout << std::endl;

    //     // torque
    //     //     for(int j(0); j<gripdata_.n; j++)               
    //     //         std::printf("% 6.2f, % 6.2f, % 6.2f /", fsuc_i[6*j], fsuc_i[6*j+1], fsuc_i[6*j+2]);
    //     //     std::cout << std::endl;
    // }

    std::cout<<" checkMaximumGraspCapability "<< std::endl;
    /*--------------------------------------------------------*/
    // the maximum box weight by this trajectory
    double m_max = 100.;
    double m1 = 1; // unit mass 

    // suction forces represented in tool frame  
    Eigen::VectorXd fts = gripdata_.G * gripdata_.fs; 
    Eigen::MatrixXd T_ot = rossy_utils::InverseSE3(
            Eigen::MatrixXd::Identity(3,3), gripdata_.p);
    Eigen::MatrixXd AdT_ot_tr = (rossy_utils::Ad_T(T_ot)).transpose();
    Eigen::VectorXd w = Eigen::VectorXd::Zero(6*gripdata_.n);
    
    for(int i(0);i<gripdata_.n;++i){
        // assume force distribution around the suction cup 
        double r = 0.95*gripdata_.rpads(i); // 0.95 < 1: safety factor
        w.segment(i*6,6) << 0.5*r*r, 0.5*r*r, r*r, 1., 1., 1.;
    }
    Eigen::MatrixXd WG = w.asDiagonal() * gripdata_.G.transpose();
    Eigen::MatrixXd GWG = gripdata_.G * w.asDiagonal() * gripdata_.G.transpose();
    Eigen::LLT<Eigen::MatrixXd> ldlt;
    ldlt.compute(GWG);


    Eigen::VectorXd fsuc_tl = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd a,b,c, tmp1, tmp2, m_suc, b_suc, g_suc;
    Eigen::MatrixXd px, wx;
    for(int k(0); k<num_wpts; ++k){
        if(k==num_wpts-1){
            dsk = sysdata_.s[k] - sysdata_.s[k-1];
            uk = 0.;
        }else{
            dsk = sysdata_.s[k+1] - sysdata_.s[k];
            uk = (x0_list[k+1] - x0_list[k]) / (2*dsk);                 
        }
        
        px = rossy_utils::skew(gripdata_.p);
        wx = rossy_utils::skew(sysdata_.ee_w[k]);

        tmp1 = gripdata_.I1*(sysdata_.ee_aw2[k]);
        tmp2 = m1*(sysdata_.ee_av2[k] + px*sysdata_.ee_aw2[k]);
        a = rossy_utils::vStack(tmp1, tmp2);
        tmp1 = gripdata_.I1*sysdata_.ee_aw1[k] + wx*gripdata_.I1*sysdata_.ee_w[k];
        tmp2 = m1*(sysdata_.ee_av1[k] + px*sysdata_.ee_aw1[k]);
        b = rossy_utils::vStack(tmp1, tmp2);
        tmp1 = Eigen::VectorXd::Zero(3);
        tmp2 = -m1*sysdata_.ee_grav[k];
        c = rossy_utils::vStack(tmp1, tmp2);

        // Mapping the equations to tool frame        
        a = AdT_ot_tr*a;
        b = AdT_ot_tr*b;
        c = AdT_ot_tr*c;

        m_suc = WG * (ldlt.solve(a));
        b_suc = WG * (ldlt.solve(b));
        g_suc = WG * (ldlt.solve(c));

        fsuc_i = uk*m_suc + x0_list[k]*b_suc + g_suc;
        fsuc_tl = gripdata_.G * fsuc_i;

        // suction loss
        // find max m s.t.
        // m * gripdata_.U * fsuc_i < gripdata_.U * gripdata_.fs
        for(int i(0); i<gripdata_.U.rows(); i++){
            double left = gripdata_.U.row(i).dot(fsuc_i);
            double right = gripdata_.U.row(i).dot(gripdata_.fs);

            if( left > 0 )
                m_max = std::min(m_max, right/left);
        }

        // traction loss    
        // find max m s.t.    
        // m * gripdata_.Ut * fsuc_tl < gripdata_.Ut * fts  
    }
    std::cout<<" given m = "<< gripdata_.m << ", max_m = "<< m_max <<std::endl;
}

Eigen::VectorXd ToptSolver::computeJerk(
        int k, const std::vector<double>& x0_list){
        
        // k: 0 ~ nwpts-2
        double dsk = sysdata_.s[k+1]-sysdata_.s[k];
        double dskk = sysdata_.s[k+2]-sysdata_.s[k+1];
        double vk0, vk1, vk2;
        vk0 = sqrt(std::max(MIN_SDOT_, x0_list[k]));
        vk1 = sqrt(std::max(MIN_SDOT_, x0_list[k+1]));
        vk2 = sqrt(std::max(MIN_SDOT_, x0_list[k+2]));
        double t = dskk / (vk2+vk1) + dsk /(vk1+vk0);
        double t2 = 2.* dsk /(vk1+vk0);

        Eigen::VectorXd  J0, J1, J2;
        J0 = sysdata_.dq[k]/2./dsk - sysdata_.ddq[k];
        J1 = -sysdata_.dq[k+1]/2./dskk - sysdata_.dq[k]/2./dsk + sysdata_.ddq[k+1];
        J2 = sysdata_.dq[k+1]/2./dskk;
        
        return (J0*x0_list[k] + J1*x0_list[k+1] + J2*x0_list[k+2])/t;
}

double ToptSolver::checkLimits(const std::vector<double>& x0_list){
    double motion_in_limit = 0.0;
    int num_wpts = sysdata_.q.size();    
    int dim_robot = sysdata_.q[0].size(); 
 
    Eigen::VectorXd qdotk2, qddotk, qddotk_pre, qdddotk, trqk;
    double dsk, uk, dtk;

    // smax, t, vel, acc, torque, jerk, grasping
    Eigen::MatrixXd check_limits = Eigen::MatrixXd::Zero(num_wpts,7);
    double temp, duration(0.);
    Eigen::VectorXd fsuc_i = Eigen::VectorXd::Zero(42);
    double grineq(0.);

    for(int k(0); k<num_wpts; ++k){
        // 1st - velocity : qdot = q'*sdot
        qdotk2 = sysdata_.dq[k].cwiseProduct(sysdata_.dq[k]) * x0_list[k];
        rossy_utils::saveVector(qdotk2, "topt/qdotk2");        
        
        // 2nd
        if(k==num_wpts-1){
            dsk = sysdata_.s[k] - sysdata_.s[k-1];
            uk = 0.;
            dtk = 2.*dsk / (sqrt(x0_list[k])+sqrt(x0_list[k-1]));
        }else{
            dsk = sysdata_.s[k+1] - sysdata_.s[k];
            uk = (x0_list[k+1] - x0_list[k]) / (2*dsk);  
            dtk = 2.*dsk / (sqrt(x0_list[k+1])+sqrt(x0_list[k]));               
        }
        
        // 2nd - acceleration
        qddotk = sysdata_.ddq[k]*(x0_list[k]) + sysdata_.dq[k]*uk;
        rossy_utils::saveVector(qddotk, "topt/qacc");        
        // 2nd - trqk = m dds + b ds2 + g
        trqk = sysdata_.m[k]*uk + sysdata_.b[k]*x0_list[k] + sysdata_.g[k];
        rossy_utils::saveVector(trqk, "topt/trq");        

        // 3rd - jerk                
        if(k==0) qdddotk = 2.*qddotk/dtk;
        else if(k==num_wpts-1) qdddotk = -2.*qddotk_pre/dtk;
        else qdddotk = computeJerk(k-1, x0_list);
        rossy_utils::saveVector(qdddotk, "topt/jerk");
        // if(k==0) qddotk_pre = qddotk;
        // qdddotk = (qddotk - qddotk_pre)/dtk;
        qddotk_pre = qddotk;

        // grasping
        grineq = 0.;
        if(grasping_activated_){
            fsuc_i = sysdata_.m_suc[k]*uk 
                + sysdata_.b_suc[k]*x0_list[k] 
                + sysdata_.g_suc[k];
            rossy_utils::saveVector(trqk, "topt/fsuc_i");
            for(int i_suc(0); i_suc<gripdata_.n; ++i_suc){
                grineq = std::max(grineq, fsuc_i(i_suc*6+5)/gripdata_.fs(i_suc*6+5) );
                double mufz = gripdata_.mus(i_suc)*(fsuc_i(i_suc*6+5)-gripdata_.fs(i_suc*6+5))/sqrt(2.);
                grineq = std::max(grineq, fsuc_i(i_suc*6+3)/mufz);
                grineq = std::max(grineq, fsuc_i(i_suc*6+4)/mufz);
            }
        }

        
        // fill in check_limits for print
        check_limits(k,0) = x0_list[k];
        check_limits(k,1) = dtk;
        temp = rossy_utils::getMaxRatioValue(qdotk2, sysdata_.vm2[k]);
        check_limits(k,2) = sqrt(temp);
        if(temp > motion_in_limit) motion_in_limit = temp;
        temp = rossy_utils::getMaxRatioValue(qddotk, sysdata_.am[k]);
        check_limits(k,3) = temp;
        if(temp > motion_in_limit) motion_in_limit = temp;
        temp = rossy_utils::getMaxRatioValue(trqk, sysdata_.tm[k]);
        check_limits(k,4) = temp;
        if(temp > motion_in_limit) motion_in_limit = temp;
        temp = rossy_utils::getMaxRatioValue(qdddotk, sysdata_.jm[k]);
        check_limits(k,5) = temp;
        if(temp > motion_in_limit) motion_in_limit = temp;        
        check_limits(k,6) = grineq;
        if(grineq > motion_in_limit) motion_in_limit = grineq;

        if(k<num_wpts-1)
            duration += dtk;
    }

    rossy_utils::pretty_print(check_limits, std::cout, "_sdot_dt_vel_acc_trq_jerk_grasp");
    std::cout<<"DHC: duration = " << duration << std::endl;

    return motion_in_limit;

}

double ToptSolver::estimateMotionPeriod(
        const std::vector<double>& x0_list){
    double t(0.);
    double xk0, xk1, dsk;
    for(int i(0); i<x0_list.size()-1; ++i){
        xk0 = sqrt(std::max(0.0, x0_list[i]));
        xk1 = sqrt(std::max(0.0, x0_list[i+1]));
        dsk = sysdata_.s[i+1]-sysdata_.s[i];
        t +=  2.*dsk/(xk0+xk1);
    }
    return t;
}