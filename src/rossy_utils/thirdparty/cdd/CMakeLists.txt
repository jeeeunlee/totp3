cmake_minimum_required(VERSION 3.0.2)
project (myCdd)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src)

include_directories(
# include
  ${PROJECT_INCLUDE_DIR}
)
find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

FILE(GLOB_RECURSE headers "*.h" "*.hpp")
FILE(GLOB_RECURSE sources "*.c" "*.cpp")

add_library (myCdd SHARED ${headers} ${sources})
target_include_directories(myCdd PUBLIC   
                      ${PROJECT_INCLUDE_DIR})
target_include_directories(myCdd PUBLIC
                      ${EIGEN3_INCLUDE_DIR}
                      ${PROJECT_INCLUDE_DIR} )


#install(TARGETS myCdd DESTINATION "${INSTALL_LIB_DIR}")
#install(FILES ${headers} DESTINATION
#    "${INSTALL_INCLUDE_DIR}/Geometry/ThirdParty/CDD/src")
