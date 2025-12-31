cmake_minimum_required(VERSION 3.14)
project(sim_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

ament_python_install_package(sim_node)
ament_python_install_package(keyboard_controls)

install(PROGRAMS
  sim_node/main.py
  # utility
  util_nodes/keyboard_controls.py
  util_nodes/tf_tree_helper.py
  DESTINATION lib/${PROJECT_NAME}  
)

# install zero copy forwarder 
add_executable(zero_copy_forwarder util_nodes/zero_copy_forwarder.cpp)
target_compile_features(zero_copy_forwarder PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS zero_copy_forwarder
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# important, install resources like models
install(DIRECTORY
  resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

ament_package()