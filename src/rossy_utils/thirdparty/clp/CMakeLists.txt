cmake_minimum_required(VERSION 3.0.2)
project (clpwrapper)

include_directories(
# include
  ${PROJECT_SOURCE_DIR}
)

find_package(Eigen3)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
)


FILE(GLOB_RECURSE headers "*.hpp")
FILE(GLOB_RECURSE sources *.cpp)
add_library (clpwrapper SHARED ${headers} ${sources} ${libCoinUtils} ${libClp})
target_link_libraries(clpwrapper)
target_include_directories(clpwrapper PUBLIC
            ${EIGEN3_INCLUDE_DIR}
            ${PROJECT_SOURCE_DIR} )

install(TARGETS clpwrapper DESTINATION "${INSTALL_SHAREDLIB_DIR}")
