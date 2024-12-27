
#include <math/natural_vcquartic_splines.hpp>
#include <io/io_utilities.hpp>
#include <math.h>
#include <algorithm>

NaturalVCQuarticSplines::NaturalVCQuarticSplines(){
    initialize();    
}

void NaturalVCQuarticSplines::initialize(){
    computed = false;
    ts.clear();
    ys.clear();  
    dys.clear();  
}

void NaturalVCQuarticSplines::push_back(
        double t, double y, double ydot){
    computed=false;
    ts.push_back(t);
    ys.push_back(y);
    dys.push_back(ydot);
}

void NaturalVCQuarticSplines::compute(){
    n_wpts = ys.size() -1; 

    // build hs 
    hs.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        hs[i] = ts[i+1] - ts[i];

    // build the thick upper triangular system
    // optimization based method: min z'Wz s.t. Az = u
    Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts);
    Eigen::VectorXd z = Eigen::VectorXd::Zero(n_wpts+1);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n_wpts, n_wpts+1);    
    for(int i(0); i<n_wpts; ++i){
        u(i) = 6.*(dys[i+1]+dys[i])/hs[i] 
                - 12.*(ys[i+1] - ys[i])/hs[i]/hs[i];
        A(i,i) = -1.;
        A(i,i+1) = 1.;
    }
    Eigen::VectorXd w = Eigen::VectorXd::Constant(n_wpts+1,1.);
    w(0) = 10.;
    w(n_wpts) = 10.;
    Eigen::MatrixXd AAT = A*w.asDiagonal()*A.transpose();
    z = AAT.ldlt().solve(u);
    z = w.asDiagonal()*A.transpose()*z;

    // z[end]=0, z[i+1]-z[i] = u[i]
    // Eigen::VectorXd u = Eigen::VectorXd::Zero(n_wpts);
    // Eigen::VectorXd z = Eigen::VectorXd::Zero(n_wpts+1);
    // for(int i(0); i<n_wpts; ++i){
    //     u(i) = 6.*(dys[i+1]+dys[i])/hs[i] 
    //             - 12.*(ys[i+1] - ys[i])/hs[i]/hs[i];
    // }
    // for(int i(n_wpts); i>0; --i){
    //     z(i-1) = z(i)-u(i-1);
    // }

    // build zs
    zs.resize(n_wpts+1); 
    for(int i(0); i<n_wpts+1; ++i)
        zs[i] = z(i);

    // build Cs,Ds,Es
    Cs.resize(n_wpts);  
    for(int i(0); i<n_wpts; ++i)
        Cs[i] = 0.5*zs[i]*hs[i]*hs[i];

    Ds.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        Ds[i] = 4.*(ys[i+1] - ys[i]) 
            - (dys[i+1]+3.*dys[i])*hs[i] 
            - zs[i]*hs[i]*hs[i];

    Es.resize(n_wpts);
    for(int i(0); i<n_wpts; ++i)
        Es[i] = -3*(ys[i+1] - ys[i]) 
            + (dys[i+1]+2.*dys[i])*hs[i] 
            + 0.5*zs[i]*hs[i]*hs[i];

    computed=true;
}

double NaturalVCQuarticSplines::evaluate(const double & t_in){
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return ys[0];    
    else if(i<n_wpts){
        double t = (t_in - ts[i])/(ts[i+1] - ts[i]);
        s = ys[i]
            + (dys[i]*hs[i])*t
            + Cs[i]*t*t
            + Ds[i]*t*t*t  
            + Es[i]*t*t*t*t;
        return s;        
    }
    else return ys[n_wpts];
}

double NaturalVCQuarticSplines::evaluateFirstDerivative(const double & t_in){
    if(!computed) compute();

    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;
    else if(i<n_wpts){
        double t = (t_in - ts[i])/(ts[i+1] - ts[i]);
        sdot = (dys[i]*hs[i])
            + 2.*Cs[i]*t
            + 3.*Ds[i]*t*t  
            + 4.*Es[i]*t*t*t;
        return sdot/hs[i];   
    }
    else return 0.;      
}

double NaturalVCQuarticSplines::evaluateSecondDerivative(const double & t_in){
    if(!computed) compute();
    
    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;
    else if(i<n_wpts){
        double t = (t_in - ts[i])/(ts[i+1] - ts[i]);
        sddot = 2.*Cs[i]
            + 6.*Ds[i]*t  
            + 12.*Es[i]*t*t;
        return sddot/hs[i]/hs[i];
    }
    else return 0.;
    
}

int NaturalVCQuarticSplines::evaluateTimeInterval(const double & t_in){
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

NVCQSpln4Vec::NVCQSpln4Vec(){
    initialize(0);
}

void NVCQSpln4Vec::initialize(int _dim){
    computed=false;

    dim = _dim;    
    curves.clear();
    for(int i(0); i<dim; ++i){
        curves.push_back( NaturalVCQuarticSplines() );
        curves[i].initialize();
    }
    output = Eigen::VectorXd::Zero(dim);
}
void NVCQSpln4Vec::push_back(double t, 
            const Eigen::VectorXd& Y, 
            const Eigen::VectorXd& YDot){
    assert(Y.size() == dim);
    computed=false;    
    for(int i(0); i<dim; ++i){
        curves[i].push_back(t, Y(i), YDot(i));
    }
}
void NVCQSpln4Vec::compute(){
    computed = true;
    for(int i(0); i<dim; ++i)
        curves[i].compute();
}

Eigen::VectorXd NVCQSpln4Vec::evaluate(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluate(t_in);
    return output;
}   

Eigen::VectorXd NVCQSpln4Vec::evaluateFirstDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateFirstDerivative(t_in);
    return output;
}

Eigen::VectorXd NVCQSpln4Vec::evaluateSecondDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateSecondDerivative(t_in);
    return output;
}

int NVCQSpln4Vec::evaluateTimeInterval(const double & t_in){
    return curves[0].evaluateTimeInterval(t_in);
}