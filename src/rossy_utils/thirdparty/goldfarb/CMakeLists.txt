cmake_minimum_required(VERSION 3.0.2)
project (Goldfarb)

include_directories(
# include
  ${PROJECT_SOURCE_DIR}
)

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)

FILE(GLOB headers "*.hh" "*.h" "*.hpp")
FILE(GLOB_RECURSE sources *.cc)
add_library (myGoldfarb SHARED ${headers} ${sources})

install(TARGETS myGoldfarb DESTINATION "${INSTALL_SHAREDLIB_DIR}")
# install(FILES ${headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/thirdparty/Goldfarb")
