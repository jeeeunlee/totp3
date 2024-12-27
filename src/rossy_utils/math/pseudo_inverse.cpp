#include "math/pseudo_inverse.hpp"
#include <Eigen/LU>
#include <Eigen/SVD>
#include <stdio.h>
#include <iostream>
using namespace std;

namespace rossy_utils {

    void pseudoInverse(Eigen::MatrixXd const & matrix,
                       double sigmaThreshold,
                       Eigen::MatrixXd & invMatrix,
                       Eigen::VectorXd * opt_sigmaOut)    {

        if ((1 == matrix.rows()) && (1 == matrix.cols())) {
            // workaround for Eigen2
            invMatrix.resize(1, 1);
            if (matrix.coeff(0, 0) > sigmaThreshold) {
                invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
            }
            else {
                invMatrix.coeffRef(0, 0) = 0.0;
            }
            if (opt_sigmaOut) {
                opt_sigmaOut->resize(1);
                opt_sigmaOut->coeffRef(0) = matrix.coeff(0, 0);
            }
            return;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // not sure if we need to svd.sort()... probably not
        int const nrows(svd.singularValues().rows());

        Eigen::MatrixXd invS;
        double sii;
        invS = Eigen::MatrixXd::Zero(nrows, nrows);
        for (int ii(0); ii < nrows; ++ii) {
            sii = svd.singularValues().coeff(ii) ;
            if (sii > 2*sigmaThreshold) 
                invS.coeffRef(ii, ii) = 1.0 / sii;            
            else
                invS.coeffRef(ii, ii) = sii / 
                    (sii*sii + sigmaThreshold*sigmaThreshold);
            
        }
        invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();

        // Eigen::VectorXd ss = svd.singularValues();        
        // Eigen::MatrixXd V = svd.matrixV();
        // Eigen::MatrixXd U = svd.matrixU();
        // Eigen::MatrixXd J = svd.matrixU() * ss.asDiagonal() * svd.matrixV().transpose();

        // std::cout<<"J = " << std::endl;
        // for(int ii(0); ii<J.rows(); ++ii){
        //     for(int kk(0); kk<J.cols(); ++kk){
        //         std::cout<<J(ii,kk)<<", ";
        //     }
        //     std::cout<<std::endl;
        // }

        // std::cout<<"sigmas = " << std::endl;
        // for(int kk(0); kk<ss.size(); ++kk){
        //         std::cout<<ss(kk)<<", ";
        // }
        // std::cout<<std::endl;

        // std::cout<<"V = " << std::endl;
        // for(int ii(0); ii<V.rows(); ++ii){
        //     for(int kk(0); kk<V.cols(); ++kk){
        //         std::cout<<V(ii,kk)<<", ";
        //     }
        //     std::cout<<std::endl;
        // }

        // std::cout<<"U = " << std::endl;
        // for(int ii(0); ii<U.rows(); ++ii){
        //     for(int kk(0); kk<U.cols(); ++kk){
        //         std::cout<<U(ii,kk)<<", ";
        //     }
        //     std::cout<<std::endl;
        // }

        // std::cout<<"invMatrix = " << std::endl;
        // for(int ii(0); ii<invMatrix.rows(); ++ii){
        //     for(int kk(0); kk<invMatrix.cols(); ++kk){
        //         std::cout<<invMatrix(ii,kk)<<", ";
        //     }
        //     std::cout<<std::endl;
        // }
        
        if (opt_sigmaOut) {
            *opt_sigmaOut = svd.singularValues();
        }
    }

    Eigen::MatrixXd getNullSpace(const Eigen::MatrixXd & J,
                                 const double threshold) {

        Eigen::MatrixXd ret(J.cols(), J.cols());
        Eigen::MatrixXd J_pinv;
        pseudoInverse(J, threshold, J_pinv);
        ret = Eigen::MatrixXd::Identity(J.cols(), J.cols()) - J_pinv * J;
        return ret;
    }

    void weightedInverse(const Eigen::MatrixXd & J,
                         const Eigen::MatrixXd & Winv,
                         Eigen::MatrixXd & Jinv) {
            Eigen::MatrixXd lambda(J* Winv * J.transpose());
            Eigen::MatrixXd lambda_inv;
            rossy_utils::pseudoInverse(lambda, 0.0001, lambda_inv);
            Jinv = Winv * J.transpose() * lambda_inv;
    }

}
