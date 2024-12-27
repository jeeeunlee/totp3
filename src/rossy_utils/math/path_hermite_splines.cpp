#include "math/path_hermite_splines.hpp"
#include <io/io_utilities.hpp>

TOPPHermiteSplines::TOPPHermiteSplines() {
    initialize();
}

void TOPPHermiteSplines::initialize() {
    computed = false;
    ps.clear();
    vs.clear();
}

// Cubic Hermite Spline: 
// From https://en.wikipedia.org/wiki/Cubic_Hermite_spline#Unit_interval_(0,_1)
// p(s) = (2s^3 - 3s^2 + 1)*p1 + (-2*s^3 + 3*s^2)*p2 + (s^3 - 2s^2 + s)*v1 + (s^3 - s^2)*v2
// where 0 <= s <= 1. 

void TOPPHermiteSplines::push_back(double p, double x) {
    computed = false;
    ps.push_back(p);
    vs.push_back(x);  
}

// pdot = (6s^2 - 6s)*p1 + (-6s^2 + 6s)*p2 + (3s^2 - 4s + 1)*v1 + (3s^2 - 2s)*v2
// pddot = (12s - 6)*p1 + (-12s + 6)*p2 + (6s - 4)*v1 + (6s - 2)*v2
void TOPPHermiteSplines::compute() {
    // compute ts
    // assume pddot(tend)=0
    n_wpts = ps.size()-1;
    std::vector<double> delt_inv_ord; // dt[N-1] ~ dt[0]
    double ti, ti1, pi0, pi1, pi2, vi0, vi1, vi2;
    int i=n_wpts-1;
    ti = 6.*(ps[n_wpts]-ps[n_wpts-1])/(2.*vs[n_wpts-1] + 4.*vs[n_wpts]);
    delt_inv_ord.push_back(ti);  

    while(i-- > 0){
        pi0 = ps[i]; vi0 = vs[i];
        pi1 = ps[i+1]; vi1 = vs[i+1];
        pi2 = ps[i+2]; vi2 = vs[i+2];
        ti1 = ti;                      
        ti = (6.*pi2-6.*pi0 - (4.*vi1 + 2.*vi2)*ti1) / (2.*vi0+4.*vi1);
        if(ti < 0){
            std::cout <<"!!!! ----- TOPPHermiteSplines error ----- !!!!" <<std::endl;
            ti = 1e-8;
        }            
        delt_inv_ord.push_back(ti);
    }

    // accum dt to ts
    ti = 0;
    ts.clear();   
    ts.push_back(ti);
    for(auto it = delt_inv_ord.rbegin(); it != delt_inv_ord.rend(); ++it){
        ti += (*it);
        ts.push_back(ti);
    }
    computed = true;
}

void TOPPHermiteSplines::check(){
    if(!computed) return;

    std::cout<<" ---TOPPHermiteSplines : total period = "<< ts[n_wpts]<<std::endl;

    rossy_utils::pretty_print(ts,"\nts");
    rossy_utils::pretty_print(ps,"\nps");
    rossy_utils::pretty_print(vs,"\nvs");

    rossy_utils::saveVector(ts, "dhc_data/ts");
    rossy_utils::saveVector(ps, "dhc_data/ps");
    rossy_utils::saveVector(vs, "dhc_data/vs");

}

double TOPPHermiteSplines::getPeroid() {
    if(computed)
        return ts[n_wpts];
    else
        return 0.;
}

void TOPPHermiteSplines::getPeroids(std::vector<double> &_ts) {
    if(computed)
        _ts = ts;
    else
        _ts.clear();
}
// p(s) = (2s^3 - 3s^2 + 1)*p1 + (-2*s^3 + 3*s^2)*p2 + (s^3 - 2s^2 + s)*v1 + (s^3 - s^2)*v2
double TOPPHermiteSplines::evaluate(const double & t_in) {
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;    
    else if(i<n_wpts){
        s = (t_in - ts[i])/(ts[i+1] - ts[i]);
        p = (2*s*s*s - 3.*s*s + 1.)*ps[i] + (-2.*s*s*s + 3.*s*s)*ps[i+1] 
                    + (s*s*s - 2.*s*s + s)*vs[i] + (s*s*s - s*s)*vs[i+1];
        return p;
    }
    else return ps[n_wpts];
}

// pdot = (6s^2 - 6s)*p1 + (-6s^2 + 6s)*p2 + (3s^2 - 4s + 1)*v1 + (3s^2 - 2s)*v2
double TOPPHermiteSplines::evaluateFirstDerivative(const double & t_in) {
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;    
    else if(i<n_wpts){
        s = (t_in - ts[i])/(ts[i+1] - ts[i]);
        pdot = (6.*s*s - 6.*s)*ps[i] + (-6.*s*s + 6.*s)*ps[i+1] 
                + (3.*s*s - 4.*s + 1)*vs[i] + (3.*s*s - 2.*s)*vs[i+1];
        return pdot;        
    }
    else return 0.;
}
// pddot = (12s - 6)*p1 + (-12s + 6)*p2 + (6s - 4)*v1 + (6s - 2)*v2
double TOPPHermiteSplines::evaluateSecondDerivative(const double & t_in) {
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;    
    else if(i<n_wpts){
        s = (t_in - ts[i])/(ts[i+1] - ts[i]);
        pddot = (12.*s - 6.)*ps[i]  + (-12.*s + 6.)*ps[i+1] 
                + (6.*s - 4.)*vs[i] + (6.*s - 2.)*vs[i+1];
        return pddot;        
    }
    else return 0.;
}

int TOPPHermiteSplines::evaluateTimeInterval(const double & t_in) {
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

TOPPHSpln4Vec::TOPPHSpln4Vec(){
    initialize(0);
}

void TOPPHSpln4Vec::initialize(int _dim){
    computed=false;

    dim = _dim;    
    curves.clear();
    for(int i(0); i<dim; ++i){
        curves.push_back( TOPPHermiteSplines() );
        curves[i].initialize();
    }
    output = Eigen::VectorXd::Zero(dim);
}
void TOPPHSpln4Vec::push_back(const Eigen::VectorXd& P, 
                            const Eigen::VectorXd& X){
    assert(P.size() == dim);
    assert(X.size() == dim);
    computed=false;    
    for(int i(0); i<dim; ++i){
        curves[i].push_back(P(i), X(i));
    }
}
void TOPPHSpln4Vec::compute(){    
    for(int i(0); i<dim; ++i)
        curves[i].compute();
    computed = true;
}

Eigen::VectorXd TOPPHSpln4Vec::evaluate(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluate(t_in);
    return output;
}   

Eigen::VectorXd TOPPHSpln4Vec::evaluateFirstDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateFirstDerivative(t_in);
    return output;
}

Eigen::VectorXd TOPPHSpln4Vec::evaluateSecondDerivative(const double & t_in){
    output = Eigen::VectorXd::Zero(dim);
    for(int i(0); i<dim; ++i)
        output[i] = curves[i].evaluateSecondDerivative(t_in);
    return output;
}

