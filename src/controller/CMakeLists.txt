cmake_minimum_required(VERSION 3.0.2)
project(my_controller)

set(CMAKE_BUILD_TYPE "Release")
# set(PYTHON_EXECUTABLE "/usr/bin/python3")

file(GLOB_RECURSE my_robot_system_sources "my_robot_system/*.cpp")
file(GLOB_RECURSE my_robot_system_headers "my_robot_system/*.hpp")

file(GLOB_RECURSE dhc_sources "dhc/*.cpp")
file(GLOB_RECURSE dhc_headers "dhc/*.hpp")

# my robot system
message(STATUS "================")
message(STATUS ${PINOCCHIO_LIBRARIES})
message(STATUS ${PINOCCHIO_INCLUDE_DIRS})
message(STATUS "================")

add_library(my_robot_system SHARED ${my_robot_system_sources}
                                    ${my_robot_system_headers})
target_link_libraries(my_robot_system ${PINOCCHIO_LIBRARIES} ${libpinocchio} ${libeigenpy}
                                        rossy_utils)
target_include_directories(my_robot_system PUBLIC   
                            ${PROJECT_SOURCE_DIR}
                            ${PINOCCHIO_INCLUDE_DIRS} )

# manipulator dhc
add_library(my_dhc SHARED ${dhc_sources}
                          ${dhc_headers})
target_link_libraries(my_dhc  rossy_utils
                              my_robot_system
                              ${libCoinUtils} ${libClp} )
target_include_directories(my_dhc PUBLIC   
                            ${PROJECT_SOURCE_DIR} )
   
install(TARGETS my_robot_system my_dhc DESTINATION "${INSTALL_SHAREDLIB_DIR}")

# install(TARGETS my_controller DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${my_controller_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/my_controller")
