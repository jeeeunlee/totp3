#include "math/path_paramerization_splines.hpp"
#include <io/io_utilities.hpp>

TOPPSplines::TOPPSplines() {
    initialize();
}

void TOPPSplines::initialize() {
    computed = false;
    ss.clear();
    vs.clear();
    us.clear();
}

void TOPPSplines::push_back(double s, double x, double u) {
    computed = false;
    ss.push_back(s);
    if(x>1e-8)
        vs.push_back(sqrt(x));
    else
        vs.push_back(1e-8);
    us.push_back(u);
}


void TOPPSplines::compute() {
    // way pnts except current point (min=1)
    n_wpts = ss.size() -1; 

    // get dt
    double t(0.), dt, ds;    
    as.resize(n_wpts);
    bs.resize(n_wpts);
    ts.resize(n_wpts+1);
    ts[0] = t;
    for(int i(0); i<n_wpts; ++i){
        as[i]=0.;
        bs[i]=0.;

        ds = 2.*(ss[i+1]-ss[i]);
        dt = ds/(vs[i]+vs[i+1]);

        t += dt;
        ts[i+1] = t;
    }
    computed = true;
}

// void TOPPSplines::compute() {
//     // way pnts except current point (min=1)
//     n_wpts = ss.size() -1; 

//     // get dt
//     double a,b,c, dt;    
//     as.resize(n_wpts);
//     bs.resize(n_wpts);
//     ts.resize(n_wpts+1);
//     ts[0] = 0.;
//     for(int i(0); i<n_wpts; ++i){
//         // solve a*dt^2 + b*dt + c=0
//         a = (us[i]-us[i+1])/6.;
//         b = vs[i]+vs[i+1]; // > 0
//         c = 2.*(ss[i]-ss[i+1]); // -2ds < 0

//         double detm = b*b-4.*a*c;
//         double ZCE = 1e-5;
//         // exceptional case: 
//         // (i) a = 0, (ii) b2-4ac < 0        
//         if(a<ZCE){
//             dt = -c/b;
//             as[i] = ( - (2*us[i]+us[i+1])*dt/3.
//                     + vs[i+1] - vs[i] )/dt/dt;
//             bs[i] = ( (us[i]+us[i+1])*dt/2. 
//                     - vs[i+1]+vs[i] )/2./dt/dt/dt;
//         }
//         else if(detm < 0){
//             a = us[i]/6.;
//             b = 2./3.*vs[i] + 1./3.*vs[i+1];
//             c = (ss[i]-ss[i+1]); // -ds
//             if( a < ZCE ){
//                 dt = -c/b;
//                 as[i] = (-us[i]*dt+vs[i+1]-vs[i])/3./dt/dt;
//                 bs[i] = 0.;
//             }else{
//                 detm = b*b-4.*a*c;
//                 dt = (-b + sqrt(detm))/2./a;
//                 as[i] = (-us[i]*dt+vs[i+1]-vs[i])/3./dt/dt;
//                 bs[i] = 0.;                
//             }
//         }
//         else{
//             dt = (-b + sqrt(detm))/2./a;
//             as[i] = ( -(2.*us[i] + us[i+1])*dt/3. 
//                     + vs[i+1]-vs[i] )/dt/dt;
//             bs[i] = ( (us[i]+us[i+1])*dt/2. 
//                     - vs[i+1]+vs[i] )/2./dt/dt/dt;
//         }
        
//         ts[i+1] = ts[i] + dt;
//     }

//     computed = true;
// }

void TOPPSplines::check(){
    if(!computed) return;

    std::cout<<" ---TOPPSplines : total period = "<< ts[n_wpts]<<std::endl;
    // int i=0;
    // for(auto &t : ts){
    //     rossy_utils::saveValue(t, "ts");

    //     double s = evaluate(t);
    //     std::cout<<"s("<<t<<") = "<<s<<", ";
    //     s=evaluateFirstDerivative(t);
        
    //     // std::cout<<"ds = "<<s<<", v["<<i<<"]="<<vs[i]<<", ";
    //     std::cout<<"v["<<i<<"]="<<vs[i]<<", ";

    //     s=evaluateSecondDerivative(t);
    //     // std::cout<<"dds = "<<s<<", u["<<i<<"]="<<us[i]<<std::endl;
    //     std::cout<<"u["<<i<<"]="<<us[i]<<std::endl;
    //     i++;
    // }  
    rossy_utils::pretty_print(ts,"\nts");
    rossy_utils::pretty_print(ss,"\nss");
    rossy_utils::pretty_print(vs,"\nvs");
    rossy_utils::pretty_print(us,"\nus");

    rossy_utils::saveVector(ts, "dhc_data/ts");
    rossy_utils::saveVector(ss, "dhc_data/ss");
    rossy_utils::saveVector(vs, "dhc_data/vs");
    rossy_utils::saveVector(us, "dhc_data/us");
}

double TOPPSplines::getPeroid() {
    if(computed)
        return ts[n_wpts];
    else
        return 0.;
}

void TOPPSplines::getPeroids(std::vector<double> &_ts) {
    if(computed)
        _ts = ts;
    else
        _ts.clear();
}

double TOPPSplines::evaluate(const double & t_in) {
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;    
    else if(i<n_wpts){
        double t = (t_in - ts[i]);
        s = ss[i] + vs[i]*t + 0.5*us[i]*t*t
             + as[i]*t*t*t + bs[i]*t*t*t*t;
        return s;        
    }
    else return ss[n_wpts];
}

double TOPPSplines::evaluateFirstDerivative(const double & t_in) {
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;    
    else if(i<n_wpts){
        double t = (t_in - ts[i]);
        sdot = vs[i] + us[i]*t + 3.*as[i]*t*t + 4.*bs[i]*t*t*t;
        return sdot;        
    }
    else return 0.;
}

double TOPPSplines::evaluateSecondDerivative(const double & t_in) {
    if(!computed) compute();
    int i = evaluateTimeInterval(t_in);
    if(i<0) return 0.;    
    else if(i<n_wpts){
        double t = (t_in - ts[i]);
        sddot = us[i] + 6.*as[i]*t + 12.*bs[i]*t*t;
        return sddot;        
    }
    else return 0.;
}

int TOPPSplines::evaluateTimeInterval(const double & t_in) {
    // return i = -1, 0 ~ (n_wpts-1), (n_wpts)
    // t_i < t_in < t_(i+1)
    int i=-1;
    for(auto &ti : ts){
        if(ti > t_in) break;
        else i++; // max: (n_wpts+1)=ts.size()
    }        
    return i;
}
