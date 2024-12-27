cmake_minimum_required(VERSION 3.5.1)

# Dexterity & HCRL Controller
project(DHC)

##=============================================================================
## CMake Flags
##=============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel" FORCE)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS on)
set(CXX_DISABLE_WERROR True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/dhc)
set(CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
set(PYTHON_BINDING TRUE)

##=============================================================================
## Build Setup
##=============================================================================

set(CMAKE_INSTALL_PREFIX "/home/jelee/dexrobo/customer/fedex/truck-loading/dhc")
set(INSTALL_LIB_DIR ${CMAKE_INSTALL_PREFIX} )
set(INSTALL_SHAREDLIB_DIR ${CMAKE_INSTALL_PREFIX}/sharedlib )
set(INSTALL_BIN_DIR ${CMAKE_INSTALL_PREFIX}/bin )
set(INSTALL_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/)
# set(PYTHON_BINDING TRUE)

##=============================================================================
## Configuration File
##=============================================================================
set(SERVO_RATE 0.001)


##=============================================================================
## Include Directory
##=============================================================================
include_directories("${PROJECT_SOURCE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/build")
include_directories("/usr/local/include")
link_directories("/usr/local/lib")

##=============================================================================
## Dependancy
##=============================================================================
# eigen
if(NOT EIGEN_FOUND)
  find_package(Eigen)
endif()

# pinocchio
find_package(pinocchio)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PINOCCHIO_INCLUDE_DIR})


find_library(libpinocchio
            NAMES libpinocchio.so
            HINTS $ENV{CONDA_PREFIX}/lib
            REQUIRED)

find_library(libeigenpy
            NAMES libeigenpy.so
            HINTS $ENV{CONDA_PREFIX}/lib
            REQUIRED)

# clp
find_library(libClp
            NAMES libClp.so
            HINTS /usr/lib /usr/lib/x86_64-linux-gnu
            REQUIRED)

find_library(libCoinUtils
            libCoinUtils.so
            HINTS /usr/lib /usr/lib/x86_64-linux-gnu
            REQUIRED)

message(STATUS "================")
message(STATUS $ENV{CONDA_PREFIX})
message(STATUS ${libpinocchio})
message(STATUS ${libeigenpy})
message(STATUS ${libCoinUtils})
message(STATUS ${libClp})
message(STATUS "================")

##=============================================================================
## Add Subdirectory
##=============================================================================
set(MYYAML_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/thirdparty/myYaml/include)
set(GOLDFARB_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/thirdparty)
set(SDLP_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/thirdparty)
set(CLP_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/src/thirdparty)
#set(PINOCCHIO_PATH ${PROJECT_SOURCE_DIR}/src/thirdparty/pinocchio)


add_subdirectory(src/rossy_utils)
add_subdirectory(src/controller)
add_subdirectory(src/simulator)

# pybind11
if(PYTHON_BINDING)
  # control pc requires this snippet for some reason..
  # set(pybind11_DIR "/home/jelee/.local/lib/python3.8/site-packages/pybind11/share/cmake/pybind11")
  find_package(Python COMPONENTS Interpreter Development)
  find_package(pybind11 CONFIG REQUIRED)
  include_directories(${pybind11_INCLUDE_DIRS})
  add_subdirectory(src/bindings/dhc)
endif()


