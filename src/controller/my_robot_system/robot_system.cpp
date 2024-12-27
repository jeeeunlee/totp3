#include "my_robot_system/robot_system.hpp"
#include <rossy_utils/io/io_utilities.hpp>
#include <chrono>

RobotSystem::RobotSystem(const RobotSystem& robotsys) 
{
    rossy_utils::pretty_constructor(2, "Robot Model(temp)");
    urdf_file_ = robotsys.urdf_file_;       
    _initializeRobotInfo();
    // printRobotInfo();
}

RobotSystem::RobotSystem(const std::string& file): urdf_file_(file) {
    rossy_utils::pretty_constructor(1, "Robot Model");
    rossy_utils::color_print(myColor::BoldGreen, "    || path="+urdf_file_);
    _initializeRobotInfo();
    printRobotInfo();
}

RobotSystem::~RobotSystem() {}


Eigen::MatrixXd RobotSystem::getMassMatrix() {
    data_.M.triangularView<Eigen::StrictlyLower>() =
        data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    return data_.M;
}

Eigen::MatrixXd RobotSystem::getInvMassMatrix() {    
    data_.Minv = 0.5*(data_.Minv + data_.Minv.transpose());
    return data_.Minv;
}

Eigen::VectorXd RobotSystem::getCoriolisGravity() {
    return pinocchio::nonLinearEffects(model_, data_, q_, qdot_);
}

Eigen::MatrixXd RobotSystem::getCoriolisMatrix() {
    return computeCoriolisMatrix(model_, data_, q_, qdot_);
}

Eigen::VectorXd RobotSystem::getCoriolis() {
    return pinocchio::nonLinearEffects(model_, data_, q_, qdot_) -
         pinocchio::computeGeneralizedGravity(model_, data_, q_);
}

Eigen::VectorXd RobotSystem::getGravity() {
    return pinocchio::computeGeneralizedGravity(model_, data_, q_);
}

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(const std::string& name) {
    return this->getBodyNodeIsometry(link_idx_map_[name]); }

Eigen::Isometry3d RobotSystem::getBodyNodeIsometry(const int& link_idx) {
    Eigen::Isometry3d ret;
    const pinocchio::SE3 trans =
        pinocchio::updateFramePlacement(model_, data_, link_idx);
    ret.linear() = trans.rotation();
    ret.translation() = trans.translation();
    return ret;
}

Eigen::Matrix<double, 6, 1> RobotSystem::getBodyNodeSpatialVelocity(const std::string& name) {
    return this->getBodyNodeSpatialVelocity(link_idx_map_[name]); }

Eigen::Matrix<double, 6, 1> RobotSystem::getBodyNodeSpatialVelocity(const int& link_idx) {
  Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
  pinocchio::Motion fv = pinocchio::getFrameVelocity(
      model_, data_, link_idx, pinocchio::LOCAL_WORLD_ALIGNED);
  ret.head<3>() = fv.angular();
  ret.tail<3>() = fv.linear();
  return ret;
}

Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const std::string& name) {
    return this->getBodyNodeJacobian(link_idx_map_[name]); }


Eigen::MatrixXd RobotSystem::getBodyNodeJacobian(const int& link_idx) {
    // Analytic Jacobian
  Eigen::Matrix<double, 6, Eigen::Dynamic> jac =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  pinocchio::getFrameJacobian(model_, data_, link_idx,
                              pinocchio::LOCAL_WORLD_ALIGNED, jac);
  Eigen::Matrix<double, 6, Eigen::Dynamic> ret =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);
  return ret;
}

Eigen::VectorXd RobotSystem::getBodyNodeJacobianDotQDot(const std::string& name) {
    return this->getBodyNodeJacobianDotQDot(link_idx_map_[name]); }

Eigen::VectorXd RobotSystem::getBodyNodeJacobianDotQDot(const int& link_idx) {
    // check 
    // pinocchio::Motion fa = pinocchio::getFrameAcceleration(
    //     model_, data_, link_idx, pinocchio::LOCAL_WORLD_ALIGNED);

    // pinocchio::forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
    pinocchio::Motion fa = pinocchio::getFrameClassicalAcceleration(
        model_, data_, link_idx, pinocchio::LOCAL_WORLD_ALIGNED);

    Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
    ret.segment(0, 3) = fa.angular();
    ret.segment(3, 3) = fa.linear();

    return ret;
}

Eigen::Matrix<double, 6, 1> RobotSystem::getBodyNodeBodyVelocity(const std::string& name) {
    return this->getBodyNodeBodyVelocity(link_idx_map_[name]);
}
Eigen::Matrix<double, 6, 1> RobotSystem::getBodyNodeBodyVelocity(const int& link_idx) {
  Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
  pinocchio::Motion fv = pinocchio::getFrameVelocity(
      model_, data_, link_idx, pinocchio::LOCAL);
  ret.head<3>() = fv.angular();
  ret.tail<3>() = fv.linear();
  return ret;
}

Eigen::MatrixXd RobotSystem::getBodyNodeBodyJacobian(const std::string& name) {
    return this->getBodyNodeBodyJacobian(link_idx_map_[name]);
}

Eigen::MatrixXd RobotSystem::getBodyNodeBodyJacobian(const int& link_idx) {
  Eigen::Matrix<double, 6, Eigen::Dynamic> jac =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  pinocchio::getFrameJacobian(model_, data_, link_idx, pinocchio::LOCAL, jac);
  Eigen::Matrix<double, 6, Eigen::Dynamic> ret =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);
  return ret;
}

Eigen::VectorXd RobotSystem::getBodyNodeBodyJacobianDotQDot(const std::string& name) {
        return this->getBodyNodeBodyJacobianDotQDot(link_idx_map_[name]);
}

Eigen::VectorXd RobotSystem::getBodyNodeBodyJacobianDotQDot(const int& link_idx) {
    // pinocchio::forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
    pinocchio::Motion fa = pinocchio::getFrameClassicalAcceleration(
        model_, data_, link_idx, pinocchio::LOCAL);

    Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
    ret.segment(0, 3) = fa.angular();
    ret.segment(3, 3) = fa.linear();

    return ret;
}

int RobotSystem::getLinkIdx(const std::string& frame_name) {    
    return link_idx_map_[frame_name]; }

int RobotSystem::getJointIdx(const std::string& jointName) {    
    return joint_idx_map_[jointName]; }

std::string RobotSystem::getLinkName(const int& frame_idx) {    
    return link_idx_map_inv_[frame_idx]; }

std::string RobotSystem::getJointName(const int& joint_idx) {    
    return joint_idx_map_inv_[joint_idx]; }

void RobotSystem::updateSystem(const Eigen::VectorXd &joint_pos,
                                const Eigen::VectorXd &joint_vel) {
    // ASSUME FIXED BASE
    q_ = joint_pos;
    qdot_ = joint_vel;

    // update data
    _updateSystemData();
}

void RobotSystem::_updateSystemData(){
    pinocchio::crba(model_, data_, q_);
    pinocchio::forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
    pinocchio::computeJointJacobians(model_, data_, q_);
}

void RobotSystem::_initializeRobotInfo() {

    // set pinocchio model & data    
    std::cout << "build pinocchio" <<std::endl;
    pinocchio::urdf::buildModel(urdf_file_, model_);
    std::cout << "done" <<std::endl;
    data_ = pinocchio::Data(model_);
    n_q_ = model_.nq;
    n_qdot_ = model_.nv;
    n_dof_ = n_qdot_;

    q_ = Eigen::VectorXd::Zero(n_q_);
    qdot_ = Eigen::VectorXd::Zero(n_qdot_);
    qddot_ = Eigen::VectorXd::Zero(n_qdot_);

    // UPDATE link_idx_map_, joint_idx_map_
    for (pinocchio::FrameIndex i(0); // FrameIndex : size_t
        i < static_cast<pinocchio::FrameIndex>(model_.nframes); ++i) {
        if(model_.frames[i].type == pinocchio::FrameType::BODY){
            std::string frame_name = model_.frames[i].name;
            link_idx_map_[frame_name] = model_.getBodyId(frame_name);
            link_idx_map_inv_[model_.getBodyId(frame_name)] = frame_name;             
        }
    }
    for (pinocchio::JointIndex i(0);
        i < static_cast<pinocchio::JointIndex>(model_.njoints); ++i) {
        total_mass_ += model_.inertias[i].mass();
        std::string joint_name = model_.names[i];
        if (joint_name != "universe" && joint_name != "root_joint"){
            joint_idx_map_[joint_name] = model_.getJointId(joint_name); // i - 2; joint map excluding fixed joint
            joint_idx_map_inv_[model_.getJointId(joint_name)] = joint_name; // joint map excluding fixed joint
        }           
    }
    n_link_ = link_idx_map_.size();
    assert(n_dof_ == joint_idx_map_.size());
}

void RobotSystem::printRobotInfo() {
    
    std::cout << " ==== Body Node ====" << std::endl;
    for (auto &[idx, name] : link_idx_map_inv_) {
        std::cout << "constexpr int " << name  << " = "
                  << std::to_string(idx) << ";" << std::endl;
    }
    std::cout << " ==== DoF ====" << std::endl;
    for (auto &[idx, name] : joint_idx_map_inv_) {
        std::cout << "constexpr int " << name << " = " 
                  << std::to_string(idx) << ";" << std::endl;
    }
    std::cout << " ==== Num ====" << std::endl;
    std::cout << "constexpr int n_bodynode = "
              << std::to_string(n_link_) << ";"
              << std::endl;
    std::cout << "constexpr int n_dof = " << std::to_string(n_qdot_) << ";"
              << std::endl;

    std::cout << " ==== Just info ====" << std::endl;
    std::cout << n_link_ <<", "  // 86
            << n_q_ << ", "  // 25 7 + 12 + 6
            << n_qdot_ <<", "  // 24  6 + 12 + 6
            << n_dof_ << std::endl; // 18 = 12 + 6

}


