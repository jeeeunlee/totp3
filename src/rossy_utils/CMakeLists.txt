cmake_minimum_required(VERSION 3.0.2)
project(rossy_utils)

set(CMAKE_BUILD_TYPE "Release")
set(THIRDPARTY_INCLUDE_DIR "thirdparty")

find_package(Eigen3)
add_subdirectory(thirdparty/yaml)
add_subdirectory(thirdparty/clp)
add_subdirectory(thirdparty/cdd)

FILE(GLOB_RECURSE sources "*.cpp")
FILE(GLOB general_headers "General/*.hpp")
FILE(GLOB io_headers "rossy_utils/IO/*.hpp")
FILE(GLOB math_headers "math/*.hpp" "*.h")

add_library(rossy_utils SHARED  ${sources} ${general_headers} ${io_headers} ${math_headers})
target_link_libraries(rossy_utils yamlwrapper clpwrapper myCdd ${EIGEN_LIBRARIES} ${libCoinUtils} ${libClp})
target_include_directories(rossy_utils PUBLIC   
                      ${PROJECT_SOURCE_DIR}
                      ${EIGEN3_INCLUDE_DIR} 
                      ${THIRDPARTY_INCLUDE_DIR} )

message(status ${CMAKE_SOURCE_DIR})
message(status ${PROJECT_SOURCE_DIR})

install(TARGETS rossy_utils DESTINATION "${INSTALL_SHAREDLIB_DIR}")


SET (THIS_COM ${CMAKE_SOURCE_DIR}/ )
CONFIGURE_FILE(${PROJECT_SOURCE_DIR}/config.h.cmake ${PROJECT_SOURCE_DIR}/Configuration.h)

# install(TARGETS rossy_utils DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${general_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/utils/General")
# install(FILES ${io_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils/IO")
# install(FILES ${math_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/Utils/Math")
