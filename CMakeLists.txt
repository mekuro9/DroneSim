cmake_minimum_required(VERSION 3.8)
project(dronesim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
#find_package(dronesim_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

set(dronesim_node_SRCS
  src/Ode.cpp
  src/Nim.cpp
  src/Dynamics.cpp
  src/Ode_withCargo.cpp
)

set(dronesim_node_HDRS
  include/dronesim/Ode.h
  include/dronesim/Nim.h
  include/dronesim/Ode_withCargo.h
)

#TODO
add_executable(talker ${dronesim_node_SRCS})
ament_target_dependencies(talker rclcpp std_msgs)
#ament_target_dependencies(listener rclcpp dronesim_interfaces) 

#TODO
#add_executable(listener src/sub.cpp)
#ament_target_dependencies(listener rclcpp dronesim_interfaces) 

install(TARGETS
  talker
  #listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
