
#include <math/natural_quartic_splines.hpp>
#include <io/io_utilities.hpp>
#include <math.h>
#include <algorithm>

NaturalQuarticSplines::NaturalQuarticSplines(){
    initialize();
    quartic_spline_type = SplineType::NATURAL;
}

void NaturalQuarticSplines::initialize(){
    computed = false;
    ts.clear();
    ys.clear();    
}

void NaturalQuarticSplines::push_back(double t, double y){
    computed=false;
    ts.push_back(t);
    ys.push_back(y);
}

void NaturalQuarticSplines::compute(){
    switch (quartic_spline_type)
    {
    case SplineType::NATURAL:
        computeNatural();
        break;
    case SplineType::OPT:
        computeOpt();
        break;
    default:
        computeNatural();
        break;
    }  
    computed=true;
}


void NaturalQuarticSplines::computeOpt(){
    n_wpts = ys.size() -1; 

    // build hs 
    hs.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        hs[i] = ts[i+1] - ts[i];

    // build hhs; let hhs[k] = X[k+1]
    std::vector<double> hhs(n_wpts-1);   
    for(int i(0); i<n_wpts-1; ++i)
        hhs[i] = hs[i]+hs[i+1]; // ts[i+2] - ts[i]; // 
    
    // build bs
    std::vector<double> bs(n_wpts);   
    for(int i(0); i<n_wpts; ++i)
        bs[i] = (ys[i+1] - ys[i])/hs[i]; 

    // build the thick upper triangular system
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts-1);
    Eigen::VectorXd znC = Eigen::VectorXd::Zero(n_wpts+2);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_wpts-1, n_wpts+2);        
    for(int i(0); i<n_wpts-1; ++i){
        u(i) = 8.*(bs[i]-bs[i+1]);
        A(i,i+1) = hhs[i]*hhs[i];
        A(i,i) = hs[i]*hs[i]/3.;
        A(i,i+2) = -A(i,i);
        A(i,n_wpts+1) = 8.*hhs[i];
    }
    for(int i(0); i<n_wpts-1; ++i){
        for(int k=i+1; k<n_wpts-1; ++k)
            A(i,k+1) += 2.*hhs[i]*hhs[k];
    }

    Eigen::MatrixXd AAT = A*A.transpose();
    znC = AAT.fullPivHouseholderQr().solve(u);
    znC = A.transpose()*znC;

    // Eigen::VectorXd u2 = A*znC;
    // rossy_utils::pretty_print(u, std::cout, "u");
    // rossy_utils::pretty_print(u2, std::cout, "u2");


    // build zs
    zs.resize(n_wpts+1); 
    for(int i(0); i<n_wpts+1; ++i)
        zs[i] = znC(i);

    // build Cs,Ds,Es
    Cs.resize(n_wpts);  
    Cs[n_wpts-1]=znC(n_wpts+1);
    for(int i(n_wpts-1); i>0; --i)
        Cs[i-1] = Cs[i] + zs[i]*hhs[i-1]/4.;

    Ds.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        Ds[i] = -zs[i+1]*hs[i]*hs[i]/24. + ys[i+1]/hs[i];

    Es.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        Es[i] = zs[i]*hs[i]*hs[i]/24. + ys[i]/hs[i];

}

void NaturalQuarticSplines::computeNatural(){
    // way pnts except current point (min=1)
    n_wpts = ys.size() -1; 

    // build hs 
    hs.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        hs[i] = ts[i+1] - ts[i];

    // build hhs; let hhs[k] = X[k+1]
    std::vector<double> hhs(n_wpts-1);   
    for(int i(0); i<n_wpts-1; ++i)
        hhs[i] = hs[i]+hs[i+1]; // ts[i+2] - ts[i]; // 
    
    // build bs
    std::vector<double> bs(n_wpts);   
    for(int i(0); i<n_wpts; ++i)
        bs[i] = (ys[i+1] - ys[i])/hs[i];  
       
    // build the thick upper triangular system
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts-1);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n_wpts-1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_wpts-1, n_wpts-1);    
    for(int i(0); i<n_wpts-1; ++i){
        u(i) = 8.*(bs[i]-bs[i+1]);
        A(i,i) = hhs[i]*hhs[i];
    }
    for(int i(1); i<n_wpts-1; ++i){
        A(i,i-1) = hs[i]*hs[i]/3.;
        A(i-1,i) = 2.*hhs[i-1]*hhs[i]-A(i,i-1);
    }
    for(int i(0); i<n_wpts-2; ++i){
        for(int j=i+2; j<n_wpts-1; ++j)
            A(i,j) = 2.*hhs[i]*hhs[j];
    }

    // compute zs
    // solve the triagonal system (Az = u)
    // todo: impelment triagonal system solution      
    z = A.fullPivHouseholderQr().solve(u);

    // build zs
    zs.resize(n_wpts+1);
    zs[0] = 0.;     
    for(int i(0); i<n_wpts-1; ++i)
        zs[i+1] = z(i);
    zs[n_wpts] = 0.;

    // build Cs,Ds,Es
    Cs.resize(n_wpts);  
    Cs[n_wpts-1]=0.;
    for(int i(n_wpts-1); i>0; --i)
        Cs[i-1] = Cs[i] + zs[i]*hhs[i-1]/4.;

    Ds.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        Ds[i] = -zs[i+1]*hs[i]*hs[i]/24. + ys[i+1]/hs[i];

    Es.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        Es[i] = zs[i]*hs[i]*hs[i]/24. + ys[i]/hs[i];

    // rossy_utils::pretty_print(zs, "zs");
    // rossy_utils::pretty_print(Cs, "Cs");
    // rossy_utils::pretty_print(Ds, "Ds");
    // rossy_utils::pretty_print(Es, "Es");
    // rossy_utils::pretty_print(ys, "ys");
    // rossy_utils::pretty_print(bs, "bs");
    
}

double NaturalQuarticSplines::evaluate(const double & t_in){
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return ys[0];    
    else if(i<n_wpts){
        double t1 = (t_in - ts[i]);
        double t2 = (ts[i+1] - t_in);
        s = (zs[i+1]*t1*t1*t1*t1)/(24.*hs[i]) 
            - (zs[i]*t2*t2*t2*t2)/(24.*hs[i]) 
            + Cs[i]*t1*t2
            + Ds[i]*t1 
            + Es[i]*t2;
        return s;        
    }
    else return ys[n_wpts];
}

double NaturalQuarticSplines::evaluateFirstDerivative(const double & t_in){
    if(!computed) compute();

    int i = evaluateTimeInterval(t_in);
    if(i<0) i=0;
    else if(i<n_wpts){}
    else i=n_wpts-1;

    double t1 = (t_in - ts[i]);
    double t2 = (ts[i+1] - t_in);
    sdot = (zs[i+1]*t1*t1*t1)/(6.*hs[i]) 
            + (zs[i]*t2*t2*t2)/(6.*hs[i]) 
            + Cs[i]*(t2-t1)
            + Ds[i] 
            - Es[i];

    return sdot;     
}

double NaturalQuarticSplines::evaluateSecondDerivative(const double & t_in){
    if(!computed) compute();
    
    int i = evaluateTimeInterval(t_in);
    if(i<0) i=0;
    else if(i<n_wpts){}
    else i=n_wpts-1;

    double t1 = (t_in - ts[i]);
    double t2 = (ts[i+1] - t_in);
    sddot = zs[i+1]*t1*t1/(2.*hs[i]) 
            - zs[i]*t2*t2/(2.*hs[i]) 
            - 2.*Cs[i];
    return sddot;
}

int NaturalQuarticSplines::evaluateTimeInterval(const double & t_in){
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

NQSpln4Vec::NQSpln4Vec(){
    initialize(0);
}

void NQSpln4Vec::initialize(int _dim){
    computed=false;

    dim = _dim;    
    curves.clear();
    for(int i(0); i<dim; ++i){
        curves.push_back( NaturalQuarticSplines() );
        curves[i].initialize();
    }
    output = Eigen::VectorXd::Zero(dim);
}
void NQSpln4Vec::push_back(double t, Eigen::VectorXd Y){
    assert(Y.size() == dim);
    computed=false;    
    for(int i(0); i<dim; ++i){
        curves[i].push_back(t, Y(i));
    }
}
void NQSpln4Vec::compute(){
    computed = true;
    for(int i(0); i<dim; ++i)
        curves[i].compute();
}

Eigen::VectorXd NQSpln4Vec::evaluate(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluate(t_in);
    return output;
}   

Eigen::VectorXd NQSpln4Vec::evaluateFirstDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateFirstDerivative(t_in);
    return output;
}

Eigen::VectorXd NQSpln4Vec::evaluateSecondDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateSecondDerivative(t_in);
    return output;
}


/// -----------------------------------------
// 좀 생각해봐야겟다
// 될까?
// 확인해보기 S2


NQSpln4Rot::NQSpln4Rot(){
    initialize();
}

void NQSpln4Rot::initialize(){
    initialized = true;
    computed=false;
    log_curves.initialize(3);

}
void NQSpln4Rot::push_back(double t, Eigen::Quaterniond q_ti){
    if(initialized) q_init = q_ti;// first push
    initialized = false;
    
    Eigen::AngleAxisd delq_axis = Eigen::AngleAxisd(q_init.inverse()*q_ti);
    Eigen::VectorXd delq_vec = delq_axis.axis() * delq_axis.angle();
    log_curves.push_back(t, delq_vec);    
}
void NQSpln4Rot::compute(){
    log_curves.compute();
}

Eigen::Quaterniond NQSpln4Rot::evaluate(const double & t_in){
    Eigen::VectorXd delq_vec = log_curves.evaluate(t_in);
    if(delq_vec.norm() < 1e-6)
        delq = Eigen::Quaterniond(1, 0, 0, 0);
    else 
        delq = Eigen::Quaterniond(Eigen::AngleAxisd(
            delq_vec.norm(), delq_vec/delq_vec.norm()));
    return ( q_init * delq );
}

Eigen::Vector3d NQSpln4Rot::evaluateFirstDerivative(const double & t_in){
    return log_curves.evaluateFirstDerivative(t_in);
}

Eigen::Vector3d NQSpln4Rot::evaluateSecondDerivative(const double & t_in){
    return log_curves.evaluateSecondDerivative(t_in);
}