cmake_minimum_required(VERSION 3.0.2)
project(my_simulator)

set(CMAKE_BUILD_TYPE "Release")
# set(PYTHON_EXECUTABLE "/usr/bin/python3")


# include_directories(
# # include
#   ${PROJECT_SOURCE_DIR}
#   ${EIGEN3_INCLUDE_DIR}
# )

# file(GLOB_RECURSE sources "*.cpp")

# add_executable(run_simulator src/main.cpp )
# target_link_libraries(run_simulator 
#                       my_dhc 
#                       rossy_utils 
#                       my_robot_system
#                       ${PINOCCHIO_LIBRARIES})

# target_include_directories(run_simulator PUBLIC 
#                             ${PROJECT_SOURCE_DIR}
#                             ${PINOCCHIO_INCLUDE_DIRS})


# add_executable(run_test src/main_test.cpp )
# target_link_libraries(run_test 
#                         my_dhc 
#                         rossy_utils 
#                         my_robot_system
#                         ${PINOCCHIO_LIBRARIES})

# target_include_directories(run_test PUBLIC 
#                             ${PROJECT_SOURCE_DIR}
#                             ${PINOCCHIO_INCLUDE_DIRS})