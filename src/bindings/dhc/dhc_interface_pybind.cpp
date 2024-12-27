#include <iostream>
#include <string>

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "controller/dhc/topt/test_interface.hpp"
#include "controller/dhc/user_command.hpp"

class PyInterface : public EnvInterface {
  using EnvInterface::EnvInterface;
  void getCommand(SensorData *_sensor_data, RobotCommand *_command) override {
    PYBIND11_OVERLOAD_PURE(void, EnvInterface, getCommand, _sensor_data, _command);
  }
  bool doPlanning(void* user_cmd) override {
    PYBIND11_OVERLOAD_PURE(bool, EnvInterface, doPlanning, user_cmd);
  }
  void updateState(SensorData *_sensor_data) override {
    PYBIND11_OVERLOAD_PURE(void, EnvInterface, updateState, _sensor_data);
  }
};

namespace py = pybind11;

PYBIND11_MODULE(dhc_interface, m) {
  py::class_<TRAJ_DATA>(m, "TRAJ_DATA")
      .def(py::init<>())
      .def_readwrite("tdata", &TRAJ_DATA::tdata)
      .def_readwrite("qdata", &TRAJ_DATA::qdata)
      .def_readwrite("dqdata", &TRAJ_DATA::dqdata)
      .def_readwrite("xdata", &TRAJ_DATA::xdata)
      .def_readwrite("dxdata", &TRAJ_DATA::dxdata)
      .def_readwrite("period", &TRAJ_DATA::period)
      .def_readwrite("singularityinpath", &TRAJ_DATA::singularityinpath);

  py::class_<PLANNING_COMMAND>(m, "PLANNING_COMMAND")
        .def(py::init<>())
        .def_readwrite("joint_path", &PLANNING_COMMAND::joint_path)
        .def_readwrite("cartesian_path", &PLANNING_COMMAND::cartesian_path)
        .def_readwrite("max_joint_jerk", &PLANNING_COMMAND::max_joint_jerk)
        .def_readwrite("max_joint_acceleration", &PLANNING_COMMAND::max_joint_acceleration)
        .def_readwrite("max_joint_speed", &PLANNING_COMMAND::max_joint_speed)
        .def_readwrite("max_linear_acceleration", &PLANNING_COMMAND::max_linear_acceleration)
        .def_readwrite("max_linear_speed", &PLANNING_COMMAND::max_linear_speed)
        .def_readwrite("acc_percentage_path_ratio", &PLANNING_COMMAND::acc_percentage_path_ratio)
        .def_readwrite("acc_percentage", &PLANNING_COMMAND::acc_percentage)
        .def_readwrite("dec_percentage_path_ratio", &PLANNING_COMMAND::dec_percentage_path_ratio)
        .def_readwrite("dec_percentage", &PLANNING_COMMAND::dec_percentage);

  py::class_<WPT_DATA>(m, "WPT_DATA")
      .def(py::init<>())
      .def("getsize", &WPT_DATA::getsize)
      .def("getdata", &WPT_DATA::getdata)
      .def_readwrite("data", &WPT_DATA::data)
      .def_readwrite("b_cartesian", &WPT_DATA::b_cartesian);

  py::class_<VEC_DATA>(m, "VEC_DATA")
      .def(py::init<>())
      .def_readwrite("data", &VEC_DATA::data);  

  py::class_<SYSTEM_DATA>(m, "SYSTEM_DATA")
      .def(py::init<>())
      .def_readwrite("s", &SYSTEM_DATA::s)
      .def_readwrite("q", &SYSTEM_DATA::q)
      .def_readwrite("dq", &SYSTEM_DATA::dq)
      .def_readwrite("ddq", &SYSTEM_DATA::ddq)
      .def_readwrite("m", &SYSTEM_DATA::m)
      .def_readwrite("b", &SYSTEM_DATA::b)
      .def_readwrite("g", &SYSTEM_DATA::g)
      .def_readwrite("av", &SYSTEM_DATA::av)
      .def_readwrite("vm2", &SYSTEM_DATA::vm2)
      .def_readwrite("tm", &SYSTEM_DATA::tm)
      .def_readwrite("jm", &SYSTEM_DATA::jm)
      .def_readwrite("ee", &SYSTEM_DATA::ee)
      .def_readwrite("ee_w", &SYSTEM_DATA::ee_w)
      .def_readwrite("ee_aw1", &SYSTEM_DATA::ee_aw1)
      .def_readwrite("ee_aw2", &SYSTEM_DATA::ee_aw2)
      .def_readwrite("ee_av1", &SYSTEM_DATA::ee_av1)
      .def_readwrite("ee_av2", &SYSTEM_DATA::ee_av2)
      .def_readwrite("ee_grav", &SYSTEM_DATA::ee_grav)
      .def_readwrite("lvm", &SYSTEM_DATA::lvm)
      .def_readwrite("lam", &SYSTEM_DATA::lam);

  py::class_<EnvInterface, PyInterface>(m, "Interface")
      .def(py::init<>())
      .def("getCommand", &EnvInterface::getCommand)
      .def("doPlanning", &EnvInterface::doPlanning)
      .def("updateState", &EnvInterface::updateState);
      

  py::class_<TestInterface, EnvInterface>(m, "TestInterface")
      .def(py::init<int>())
      .def("getIKSolution", &TestInterface::getIKSolution)
      .def("getFKSolution", &TestInterface::getFKSolution)
      .def("getSysData", &TestInterface::getSysData)
      .def("getTorqueOnTrajectory", &TestInterface::getTorqueOnTrajectory)
      .def("getMaxLoadOnTrajectory", &TestInterface::getMaxLoadOnTrajectory)
      .def("getSucFrcDistOnTrajectory", &TestInterface::getSucFrcDistOnTrajectory)
      .def("getPlannedResult", &TestInterface::getPlannedResult)          
      .def("updateGripData", &TestInterface::updateGripData)
      .def("updateVelLimit", &TestInterface::updateVelLimit)
      .def("updateAccLimit", &TestInterface::updateAccLimit)
      .def("updateJerkLimit", &TestInterface::updateJerkLimit)
      .def("rePlanning", &TestInterface::rePlanning);      

  py::class_<SensorData>(m, "SensorData")
      .def(py::init<int>())
      .def_readwrite("elapsed_time", &SensorData::elapsedtime)
      .def_readwrite("joint_positions", &SensorData::q)
      .def_readwrite("joint_velocities", &SensorData::qdot);

  py::class_<RobotCommand>(m, "RobotCommand")
      .def(py::init<int>())
      .def_readwrite("joint_positions", &RobotCommand::q)
      .def_readwrite("joint_velocities", &RobotCommand::qdot)
      .def_readwrite("joint_acclerations", &RobotCommand::qddot)
      .def_readwrite("joint_torques", &RobotCommand::jtrq);

  py::class_<GRIP_DATA>(m, "GRIP_DATA")
      .def(py::init<>())
      .def("setObject", &GRIP_DATA::setObject)
      .def("setBoxObject", &GRIP_DATA::setBoxObject)
      .def_readwrite("m", &GRIP_DATA::m)
      .def_readwrite("I", &GRIP_DATA::I)
      .def_readwrite("p", &GRIP_DATA::p)
      .def_readwrite("n", &GRIP_DATA::n)
      .def_readwrite("G", &GRIP_DATA::G)
      .def_readwrite("U", &GRIP_DATA::U)
      .def_readwrite("fs", &GRIP_DATA::fs);

  py::class_<DEX_GRIPPER_V3, GRIP_DATA>(m, "DexGripperV3")
      .def(py::init<>());      

  py::class_<DEX_GRIPPER_V4, GRIP_DATA>(m, "DexGripperV4")
      .def(py::init<>());
}
