#include "math/liegroup_utilities.hpp"
#include <cassert>
#include <cmath>

namespace rossy_utils {
    Eigen::MatrixXd SE3(const Eigen::MatrixXd &R,
                        const Eigen::VectorXd &p){
        Eigen::MatrixXd Tab = Eigen::MatrixXd::Identity(4,4);
        Tab.block(0,0,3,3) = R;
        Tab.block(0,3,3,1) = p;
        return Tab;
    }
    Eigen::MatrixXd InverseSE3(const Eigen::MatrixXd &Rab,
                            const Eigen::VectorXd &pab){
        Eigen::MatrixXd Tba = Eigen::MatrixXd::Identity(4,4);
        Tba.block(0,0,3,3) = Rab.transpose();
        Tba.block(0,3,3,1) = -Rab.transpose()*pab;
        return Tba;
    }
    Eigen::MatrixXd InverseSE3(const Eigen::MatrixXd &Tab){
        Eigen::MatrixXd Rab = Tab.block(0,0,3,3);
        Eigen::VectorXd pab = Tab.block(0,3,3,1);

        return InverseSE3(Rab, pab);
    }
    Eigen::MatrixXd Ad_T(const Eigen::MatrixXd &T){
        Eigen::MatrixXd Ad_6by6 = Eigen::MatrixXd::Zero(6,6);
        Ad_6by6.block(0,0,3,3)=T.block(0,0,3,3);
        Ad_6by6.block(3,3,3,3)=T.block(0,0,3,3);
        Eigen::MatrixXd px = Eigen::MatrixXd::Zero(3,3);
        px<< 0,-T(2,3),T(1,3),
            T(2,3),0,-T(0,3),
            -T(1,3),T(0,3),0;
        Ad_6by6.block(3,0,3,3)=px*T.block(0,0,3,3);
        return Ad_6by6;
    }
}// namespace rossy_utils