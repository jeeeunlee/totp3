# pybind11_add_module(test_interface test_interface_pybind.cpp)
# target_link_libraries(test_interface PUBLIC my_dhc)

# pybind11_add_module(user_command user_command_pybind.cpp)
# target_link_libraries(user_command PUBLIC my_dhc)

pybind11_add_module(dhc_interface dhc_interface_pybind.cpp)
target_link_libraries(dhc_interface PUBLIC my_dhc)

install(TARGETS dhc_interface DESTINATION "${INSTALL_LIB_DIR}")

