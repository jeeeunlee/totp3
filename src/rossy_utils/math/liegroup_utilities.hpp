#pragma once

#include <stdio.h>
#include <Eigen/Dense>
#include <io/io_utilities.hpp>
#include <iostream>

namespace rossy_utils {
    Eigen::MatrixXd SE3(const Eigen::MatrixXd &R,
                        const Eigen::VectorXd &p);
    Eigen::MatrixXd InverseSE3(const Eigen::MatrixXd &Rab,
                               const Eigen::VectorXd &pab);
    Eigen::MatrixXd InverseSE3(const Eigen::MatrixXd &Tab);
    Eigen::MatrixXd Ad_T(const Eigen::MatrixXd &T);


}// namespace rossy_utils