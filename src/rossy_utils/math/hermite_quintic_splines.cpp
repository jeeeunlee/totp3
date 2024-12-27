
#include <math/hermite_quintic_splines.hpp>
#include <io/io_utilities.hpp>
#include <math.h>
#include <algorithm>

HermiteQuinticSplines::HermiteQuinticSplines(){
    initialize();
}

void HermiteQuinticSplines::initialize(){
    computed = false;
    ts.clear();
    ys.clear();   
    dys.clear();
    ddys.clear();
}

void HermiteQuinticSplines::push_back(double t, 
                                      double y,
                                      double ydot,
                                      double yddot){
    computed=false;
    ts.push_back(t);
    ys.push_back(y);
    dys.push_back(ydot);
    ddys.push_back(yddot);
}

// Hermite basis functions
// p(t) = p0
void HermiteQuinticSplines::compute(){

    n_wpts = ts.size()-1;
    coeffs.clear();

    // Eigen::MatrixXd H {
    //     {1., 0., 0., -10., 15., -6.},
    //     {0., 1., 0., -6., 8., -3.},
    //     {0., 0., 0.5, -1.5, 1.5, -0.5},
    //     {0., 0., 0., 0.5, -1., 0.5},
    //     {0., 0., 0., -4., 7., -3.},
    //     {0., 0., 0., 10., -15., 6.0}};
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6,6);
    H << 1., 0., 0., -10., 15., -6.,
        0., 1., 0., -6., 8., -3.,
        0., 0., 0.5, -1.5, 1.5, -0.5,
        0., 0., 0., 0.5, -1., 0.5,
        0., 0., 0., -4., 7., -3.,
        0., 0., 0., 10., -15., 6.0;
    Eigen::VectorXd ci, tmp;
    double tk;
    for(int i(0); i<n_wpts; ++i){
        tk = ts[i+1] - ts[i];
        tmp = Eigen::VectorXd::Zero(6);
        tmp << ys[i], dys[i]*tk, ddys[i]*tk*tk, 
            ddys[i+1]*tk*tk, dys[i+1]*tk, ys[i+1];
        ci = H.transpose()*tmp;        
        coeffs.push_back(ci);
    }    

    computed=true;
}

double HermiteQuinticSplines::evaluate(const double & t_in){
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return ys[0];    
    else if(i<n_wpts){        
        double t = (t_in - ts[i])/(ts[i+1] - ts[i]);
        Eigen::VectorXd ts = Eigen::VectorXd::Zero(6);
        ts << 1, t, t*t, t*t*t, t*t*t*t, t*t*t*t*t;
        double s = coeffs[i].transpose()*ts;        
        return s;        
    }
    else return ys[n_wpts];
}

double HermiteQuinticSplines::evaluateFirstDerivative(const double & t_in){
    if(!computed) compute();

    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;
    else if(i<n_wpts){
        double dtinv = 1./(ts[i+1] - ts[i]);
        double t = (t_in - ts[i])/(ts[i+1] - ts[i]);          
        Eigen::VectorXd ts = Eigen::VectorXd::Zero(6);
        ts << 0., 1., 2.*t, 3.*t*t, 4.*t*t*t, 5.*t*t*t*t;
        double sdot = (coeffs[i].transpose())*ts;

        return dtinv*sdot; 
          
    }
    else return 0.;    
}

double HermiteQuinticSplines::evaluateSecondDerivative(const double & t_in){
    if(!computed) compute();
    
    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;
    else if(i<n_wpts){
        double dtinv = 1./(ts[i+1] - ts[i]);
        double t = (t_in - ts[i])/(ts[i+1] - ts[i]);
        Eigen::VectorXd ts = Eigen::VectorXd::Zero(6);
        ts << 0., 0., 2., 6.*t, 12.*t*t, 20.*t*t*t;
        double sddot = (coeffs[i].transpose())*ts;
        return dtinv*dtinv*sddot;
    }
    else return 0.;  
    
}

int HermiteQuinticSplines::evaluateTimeInterval(const double & t_in){    
    // if ts[i] < t_in < ts[i+1]: return i = 0 ~ (n_wpts-1)
    // if t_in<ts[0]: return i = -1, 
    // if ts[n_wpts]<t_in: return i = n_wpts 
    int i=-1;
    for(auto &ti : ts){ //ts.size()=(n_wpts+1)
        if(ti > t_in) break;
        else i++; // ti <= t_in        
    }
    return i;
}


/// -----------------------------------------

HQSpln4Vec::HQSpln4Vec(){
    initialize(0);
}

void HQSpln4Vec::initialize(int _dim){
    computed=false;

    dim = _dim;    
    curves.clear();
    for(int i(0); i<dim; ++i){
        curves.push_back( HermiteQuinticSplines() );
        curves[i].initialize();
    }
    output = Eigen::VectorXd::Zero(dim);
}
void HQSpln4Vec::push_back(double t, 
                  const Eigen::VectorXd& Y,
                  const Eigen::VectorXd& Ydot, 
                  const Eigen::VectorXd& Yddot){
    assert(Y.size() == dim);
    computed=false;    
    for(int i(0); i<dim; ++i){
        curves[i].push_back(t, Y(i), Ydot(i), Yddot(i));
    }
}
void HQSpln4Vec::compute(){
    computed = true;
    for(int i(0); i<dim; ++i)
        curves[i].compute();
}

int HQSpln4Vec::evaluateTimeInterval(const double & t_in){
    return curves[0].evaluateTimeInterval(t_in);
}

Eigen::VectorXd HQSpln4Vec::evaluate(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluate(t_in);
    return output;
}   

Eigen::VectorXd HQSpln4Vec::evaluateFirstDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateFirstDerivative(t_in);
    return output;
}

Eigen::VectorXd HQSpln4Vec::evaluateSecondDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateSecondDerivative(t_in);
    return output;
}

