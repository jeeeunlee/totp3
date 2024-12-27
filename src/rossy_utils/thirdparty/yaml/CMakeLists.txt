cmake_minimum_required(VERSION 3.0.2)
project (yamlwrapper)

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

message(${PROJECT_SOURCE_DIR})
message(${PROJECT_INCLUDE_DIR})

file(GLOB_RECURSE sources "*.cpp")
file(GLOB yaml_headers "include/*.h")
file(GLOB yaml_contrib_headers "include/contrib/*.h")
file(GLOB yaml_node_headers "include/yaml/node/*.h")
file(GLOB yaml_node_detail_headers "include/yaml/node/detail/*.h")

add_library(yamlwrapper SHARED ${sources}
                          ${yaml_headers}
                          ${yaml_contrib_headers}
                          ${yaml_node_headers}
                          ${yaml_node_detail_headers})

target_include_directories(yamlwrapper PUBLIC
                          ${EIGEN3_INCLUDE_DIR}
                          ${PROJECT_INCLUDE_DIR} )

install(TARGETS yamlwrapper DESTINATION "${INSTALL_SHAREDLIB_DIR}")

# install(TARGETS yaml DESTINATION "${INSTALL_LIB_DIR}")
# install(FILES ${yaml_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/yaml/include/yaml")
# install(FILES ${yaml_contrib_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/yaml/include/yaml/contrib")
# install(FILES ${yaml_node_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/yaml/include/yaml/node")
# install(FILES ${yaml_node_detail_headers} DESTINATION
#     "${INSTALL_INCLUDE_DIR}/ExternalSource/yaml/include/yaml/node/detail")
