#pragma once
#include <string>

namespace Manipulator {
constexpr int n_bodynode = 8;
constexpr int n_dof = 7;
}  // namespace KAWASAKI

namespace ManipulatorEE {
constexpr int TOOL = 16; //wrist_3_link
constexpr int ELBOW = 10; //forearm
}

namespace ManipulatorBodyNode {
constexpr int ground = 2;
constexpr int driving_board_left = 4;
constexpr int shoulder_pan = 6;
constexpr int shoulder_lift = 8;
constexpr int forearm = 10;
constexpr int wrist_1_link = 12;
constexpr int wrist_2_link = 14;
constexpr int wrist_3_link = 16;
}  // namespace ManipulatorBodyNode

namespace ManipulatorDoF {
constexpr int jext = 1;
constexpr int j1 = 2;
constexpr int j2 = 3;
constexpr int j3 = 4;
constexpr int j4 = 5;
constexpr int j5 = 6;
constexpr int j6 = 7;
}  // namespace ManipulatorDoF

