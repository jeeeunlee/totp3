#pragma once
#include <string>

namespace Manipulator {
constexpr int n_bodynode = 7;
constexpr int n_dof = 6;
}  // namespace KAWASAKI

namespace ManipulatorEE {
constexpr int TOOL = 14; //wrist_3_link
constexpr int ELBOW = 8; //forearm
}

namespace ManipulatorBodyNode {
constexpr int ground = 2;
constexpr int shoulder_pan = 4;
constexpr int shoulder_lift = 6;
constexpr int forearm = 8;
constexpr int wrist_1_link = 10;
constexpr int wrist_2_link = 12;
constexpr int wrist_3_link = 14;
}  // namespace ManipulatorBodyNode

namespace ManipulatorDoF {
constexpr int j1 = 1;
constexpr int j2 = 2;
constexpr int j3 = 3;
constexpr int j4 = 4;
constexpr int j5 = 5;
constexpr int j6 = 6;
}  // namespace ManipulatorDoF

