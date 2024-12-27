
#include <math/natural_cubic_splines.hpp>
#include <io/io_utilities.hpp>
#include <math.h>
#include <algorithm>

NaturalCubicSplines::NaturalCubicSplines(){
    initialize();
    cubic_spline_type = SplineType::NATURAL;
}

void NaturalCubicSplines::initialize(){
    computed = false;
    ts.clear();
    ys.clear();
    n_wpts = 0;    
}

void NaturalCubicSplines::push_back(double t, double y){
    computed=false;
    ts.push_back(t);
    ys.push_back(y);
    ++n_wpts;
}

void NaturalCubicSplines::compute(){
    switch (cubic_spline_type)
    {
    case SplineType::NATURAL:
        computeNatural();
        break;
    case SplineType::SAMEJERK:
        computeSameJerk();
        break;
    case SplineType::SAMEACC:
        computeSameAcc();
        break;
    case SplineType::ZEROVEL:
        computeZeroVelClamped();
        break;
    case SplineType::OPT:
        computeOpt();
        break;
    default:
        computeNatural();
        break;
    }  
    
}

void NaturalCubicSplines::computeOpt(){
    // way pnts except current point (min=1)
    n_wpts = ys.size() -1; 

    // build hs && compute zs
    hs.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        hs[i] = ts[i+1] - ts[i];
    
    // build the triagonal system
    std::vector<double> bs(n_wpts);
   
    for(int i(0); i<n_wpts; ++i)
        bs[i] = (ys[i+1] - ys[i])/hs[i];    


    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts-1);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n_wpts+1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_wpts-1, n_wpts+1);    
    for(int i(0); i<n_wpts-1; ++i){
        u(i) = 6.0*(bs[i+1]-bs[i]);
        A(i,i+1) = 2.*(hs[i+1] + hs[i]);
    }
    for(int i(0); i<n_wpts-1; ++i){
        A(i,i) = hs[i];
        A(i,i+2) = hs[i+1];
    }

    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(n_wpts+1, n_wpts+1);  
    // for(int i(1); i<n_wpts+1; ++i){
    //     W(i-1,i) = -1.;
    //     W(i,i-1) = -1.;
    // }
    // solve the triagonal system (Az = u)
    // todo: impelment triagonal system solution  
    Eigen::MatrixXd AWAT = A*W*A.transpose();
    z = AWAT.ldlt().solve(u);
    // z = AAT.fullPivHouseholderQr().solve(u);
    z = W*A.transpose()*z;

    // Eigen::VectorXd udiff= A*z - u;
    // rossy_utils::pretty_print(udiff, std::cout, "udiff");

    // build zs
    zs.resize(n_wpts+1);    
    for(int i(0); i<n_wpts+1; ++i)
        zs[i] = z(i);

    // std::cout<<"zs : done" <<std::endl;
    computed = true;
}

void NaturalCubicSplines::computeZeroVelClamped(){
// way pnts except current point (min=1)
    n_wpts = ys.size() -1; 

    // build hs && compute zs
    hs.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        hs[i] = ts[i+1] - ts[i];
    
    // build the triagonal system
    std::vector<double> bs(n_wpts);
   
    for(int i(0); i<n_wpts; ++i)
        bs[i] = (ys[i+1] - ys[i])/hs[i];    


    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts+1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_wpts+1, n_wpts+1);

    for(int i(0); i<n_wpts-1; ++i){
        u(i+1) = 6.0*(bs[i+1]-bs[i]);
        A(i+1,i+1) = 2.*(hs[i+1] + hs[i]);
    }
    for(int i(0); i<n_wpts; ++i){
        A(i+1,i) = hs[i];
        A(i,i+1) = hs[i];
    }
    A(0,0)= -2.*hs[0];
    A(0,1)= -hs[0];
    A(n_wpts,n_wpts-1) = hs[n_wpts-1];
    A(n_wpts,n_wpts) = 2.*hs[n_wpts-1];
    u(0) = -6.*bs[0];
    u(n_wpts) = -6.*bs[n_wpts-1];

    // solve the triagonal system (Az = u)
    // todo: impelment triagonal system solution  
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n_wpts+1);
    // z = A.ldlt().solve(u); // psd or nsd A required 
    z = A.fullPivHouseholderQr().solve(u);

    // rossy_utils::pretty_print(A, std::cout, "A");
    // rossy_utils::pretty_print(u, std::cout, "u");
    // rossy_utils::pretty_print(z, std::cout, "z");

    // build zs
    zs.resize(n_wpts+1);  
    for(int i(0); i<n_wpts+1; ++i)
        zs[i] = z(i);


    // std::cout<<"zs : done" <<std::endl;
    computed = true;
}

void NaturalCubicSplines::computeSameAcc(){
// way pnts except current point (min=1)
    n_wpts = ys.size() -1; 

    // build hs && compute zs
    hs.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        hs[i] = ts[i+1] - ts[i];
    
    // build the triagonal system
    std::vector<double> bs(n_wpts);
   
    for(int i(0); i<n_wpts; ++i)
        bs[i] = (ys[i+1] - ys[i])/hs[i];    


    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts+1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_wpts+1, n_wpts+1);

    for(int i(0); i<n_wpts-1; ++i){
        u(i+1) = 6.0*(bs[i+1]-bs[i]);
        A(i+1,i+1) = 2.*(hs[i+1] + hs[i]);
    }
    for(int i(0); i<n_wpts; ++i){
        A(i+1,i) = hs[i];
        A(i,i+1) = hs[i];
    }
    A(0,0)= 1.;
    A(0,1)= -1.;
    A(n_wpts,n_wpts-1) = 1.;
    A(n_wpts,n_wpts) = -1.;

    // solve the triagonal system (Az = u)
    // todo: impelment triagonal system solution  
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n_wpts+1);
    // z = A.ldlt().solve(u); // psd or nsd A required 
    z = A.fullPivHouseholderQr().solve(u);

    // rossy_utils::pretty_print(A, std::cout, "A");
    // rossy_utils::pretty_print(u, std::cout, "u");
    // rossy_utils::pretty_print(z, std::cout, "z");

    // build zs
    zs.resize(n_wpts+1);  
    for(int i(0); i<n_wpts+1; ++i)
        zs[i] = z(i);


    // std::cout<<"zs : done" <<std::endl;
    computed = true;
}

void NaturalCubicSplines::computeSameJerk(){

    // way pnts except current point (min=1)
    n_wpts = ys.size() -1; 

    // build hs && compute zs
    hs.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        hs[i] = ts[i+1] - ts[i];
    
    // build the triagonal system
    std::vector<double> bs(n_wpts);
   
    for(int i(0); i<n_wpts; ++i)
        bs[i] = (ys[i+1] - ys[i])/hs[i];    


    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts+1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_wpts+1, n_wpts+1);

    for(int i(0); i<n_wpts-1; ++i){
        u(i+1) = 6.0*(bs[i+1]-bs[i]);
        A(i+1,i+1) = 2.*(hs[i+1] + hs[i]);
    }
    for(int i(0); i<n_wpts; ++i){
        A(i+1,i) = hs[i];
        A(i,i+1) = hs[i];
    }
    A(0,0)= hs[1];
    A(0,1)= -hs[0]-hs[1];
    A(0,2)= hs[0];
    A(n_wpts,n_wpts-2) = hs[n_wpts-1];
    A(n_wpts,n_wpts-1) = -hs[n_wpts-1]-hs[n_wpts-2];
    A(n_wpts,n_wpts) = hs[n_wpts-2];

    // solve the triagonal system (Az = u)
    // todo: impelment triagonal system solution  
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n_wpts+1);
    // z = A.ldlt().solve(u); // psd or nsd A required 
    z = A.fullPivHouseholderQr().solve(u);

    // rossy_utils::pretty_print(A, std::cout, "A");
    // rossy_utils::pretty_print(u, std::cout, "u");
    // rossy_utils::pretty_print(z, std::cout, "z");

    // build zs
    zs.resize(n_wpts+1);  
    for(int i(0); i<n_wpts+1; ++i)
        zs[i] = z(i);


    // std::cout<<"zs : done" <<std::endl;
    computed = true;
}

void NaturalCubicSplines::computeNatural(){

    // way pnts except current point (min=1)
    n_wpts = ys.size() -1; 

    // build hs && compute zs
    hs.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        hs[i] = ts[i+1] - ts[i];
    
    // build the triagonal system
    std::vector<double> bs(n_wpts);
   
    for(int i(0); i<n_wpts; ++i)
        bs[i] = (ys[i+1] - ys[i])/hs[i];    


    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts-1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_wpts-1, n_wpts-1);    
    for(int i(0); i<n_wpts-1; ++i){
        u(i) = 6.0*(bs[i+1]-bs[i]);
        A(i,i) = 2.*(hs[i+1] + hs[i]);
    }
    for(int i(1); i<n_wpts-1; ++i){
        A(i,i-1) = hs[i];
        A(i-1,i) = hs[i];
    }

    // solve the triagonal system (Az = u)
    // todo: impelment triagonal system solution  
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n_wpts-1);
    z = A.ldlt().solve(u); // psd or nsd A required 
    // z = A.fullPivHouseholderQr().solve(u);

    // rossy_utils::pretty_print(A, std::cout, "A");
    // rossy_utils::pretty_print(u, std::cout, "u");
    // rossy_utils::pretty_print(z, std::cout, "z");

    // build zs
    zs.resize(n_wpts+1);
    zs[0] = 0.;     
    for(int i(0); i<n_wpts-1; ++i)
        zs[i+1] = z(i);
    zs[n_wpts] = 0.;

    // std::cout<<"zs : done" <<std::endl;
    computed = true;
}

double NaturalCubicSplines::evaluate(const double & t_in){
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return ys[0];    
    else if(i<n_wpts){
        double t1 = (t_in - ts[i]);
        double t2 = (ts[i+1] - t_in);
        s = (zs[i+1]*t1*t1*t1)/(6.*hs[i]) 
            + (zs[i]*t2*t2*t2)/(6.*hs[i]) 
            + (ys[i+1]/hs[i] - zs[i+1]/6.*hs[i])*t1 
            + (ys[i]/hs[i] - zs[i]/6.*hs[i])*t2;
        return s;        
    }
    else return ys[n_wpts];
}

double NaturalCubicSplines::evaluateFirstDerivative(const double & t_in){
    if(!computed) compute();

    int i = evaluateTimeInterval(t_in);
    double t1,t2;
    if(i<0) {
        i=0;
        t1 = 0.;
        t2 = ts[i+1] - ts[i] - t1;
    }
    else if(i<n_wpts){
        t1 = (t_in - ts[i]);
        t2 = (ts[i+1] - t_in);
    }
    else {
        i=n_wpts-1;        
        t2 = 0.;
        t1 = ts[i+1] - ts[i] - t2;
    }
    sdot = (zs[i+1]*t1*t1)/(2.*hs[i]) 
        - (zs[i]*t2*t2)/(2.*hs[i]) 
        + (ys[i+1]/hs[i] - zs[i+1]/6.*hs[i]) 
        - (ys[i]/hs[i] - zs[i]/6.*hs[i]);
    return sdot;     
}

double NaturalCubicSplines::evaluateSecondDerivative(const double & t_in){
    if(!computed) compute();
    
    int i = evaluateTimeInterval(t_in);
    double t1,t2;
    if(i<0) {
        i=0;
        t1 = 0.;
        t2 = ts[i+1] - ts[i] - t1;
    }
    else if(i<n_wpts){
        t1 = (t_in - ts[i]);
        t2 = (ts[i+1] - t_in);
    }
    else {
        i=n_wpts-1;        
        t2 = 0.;
        t1 = ts[i+1] - ts[i] - t2;
    }

    
    sddot = (zs[i+1]*t1)/(hs[i]) 
            + (zs[i]*t2)/(hs[i]);
    return sddot;
}

int NaturalCubicSplines::evaluateTimeInterval(const double & t_in){
    // return i = -1, 0 ~ (n_wpts-1), (n_wpts)
    // t_i < t_in < t_(i+1)
    int i=-1;
    for(auto &ti : ts){
        if(ti > t_in) break;
        else i++; // max: (n_wpts+1)=ts.size()
    }        
    return i;
}


/// -----------------------------------------

NCSpln4Vec::NCSpln4Vec(){
    initialize(0);
}

void NCSpln4Vec::initialize(int _dim){
    computed=false;

    dim = _dim;    
    curves.clear();
    for(int i(0); i<dim; ++i){
        curves.push_back( NaturalCubicSplines() );
        curves[i].initialize();
    }
    output = Eigen::VectorXd::Zero(dim);
    n_wpts = 0;
}
void NCSpln4Vec::push_back(double t, Eigen::VectorXd Y){
    assert(Y.size() == dim);
    computed=false;    
    for(int i(0); i<dim; ++i){
        curves[i].push_back(t, Y(i));
    }
    ++n_wpts;
}
void NCSpln4Vec::compute(){
    computed = true;
    for(int i(0); i<dim; ++i)
        curves[i].compute();
}

Eigen::VectorXd NCSpln4Vec::evaluate(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluate(t_in);
    return output;
}   

Eigen::VectorXd NCSpln4Vec::evaluateFirstDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateFirstDerivative(t_in);
    return output;
}

Eigen::VectorXd NCSpln4Vec::evaluateSecondDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateSecondDerivative(t_in);
    return output;
}

int NCSpln4Vec::evaluateTimeInterval(const double & t_in){
    return curves[0].evaluateTimeInterval(t_in);
}

/// -----------------------------------------
// 좀 생각해봐야겟다
// 될까?
// 확인해보기 S2


NCSpln4Rot::NCSpln4Rot(){
    initialize();
}

void NCSpln4Rot::initialize(){
    initialized = true;
    computed=false;
    log_curves.initialize(3);

}
void NCSpln4Rot::push_back(double t, Eigen::Quaterniond q_ti){
    if(initialized) q_init = q_ti;// first push
    initialized = false;
    
    Eigen::AngleAxisd delq_axis = Eigen::AngleAxisd(q_init.inverse()*q_ti);
    Eigen::VectorXd delq_vec = delq_axis.axis() * delq_axis.angle();
    log_curves.push_back(t, delq_vec);    
}
void NCSpln4Rot::compute(){
    log_curves.compute();
}

Eigen::Quaterniond NCSpln4Rot::evaluate(const double & t_in){
    Eigen::VectorXd delq_vec = log_curves.evaluate(t_in);
    if(delq_vec.norm() < 1e-6)
        delq = Eigen::Quaterniond(1, 0, 0, 0);
    else 
        delq = Eigen::Quaterniond(Eigen::AngleAxisd(
            delq_vec.norm(), delq_vec/delq_vec.norm()));
    return ( q_init * delq );
}

Eigen::Vector3d NCSpln4Rot::evaluateFirstDerivative(const double & t_in){
    return log_curves.evaluateFirstDerivative(t_in);
}

Eigen::Vector3d NCSpln4Rot::evaluateSecondDerivative(const double & t_in){
    return log_curves.evaluateSecondDerivative(t_in);
}