cmake_minimum_required(VERSION 3.5)
project(lynxmotion_ssc32u_controllers)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lynxmotion_ssc32u_msgs REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(servo_controller SHARED
  src/servo_controller.cpp)
target_compile_definitions(servo_controller
  PRIVATE "LYNXMOTION_SSC32U_BUILDING_DLL")
ament_target_dependencies(servo_controller
  "rclcpp"
  "rclcpp_components"
  "std_srvs"
  "sensor_msgs"
  "trajectory_msgs"
  "lynxmotion_ssc32u_msgs")
rclcpp_components_register_nodes(servo_controller "lynxmotion_ssc32u_controllers::ServoController")
set(node_plugins "${node_plugins}lynxmotion_ssc32u_controllers::ServoController;$<TARGET_FILE:servo_controller>\n")

add_executable(servo_controller_node
  src/servo_controller_node.cpp
)
target_link_libraries(servo_controller_node
  servo_controller
)

add_library(sabertooth_2x5_controller SHARED
  src/sabertooth_2x5_controller.cpp)
target_compile_definitions(sabertooth_2x5_controller
  PRIVATE "LYNXMOTION_SSC32U_BUILDING_DLL")
ament_target_dependencies(sabertooth_2x5_controller
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "lynxmotion_ssc32u_msgs")
rclcpp_components_register_nodes(sabertooth_2x5_controller "lynxmotion_ssc32u_controllers::Sabertooth2x5Controller")
set(node_plugins "${node_plugins}lynxmotion_ssc32u_controllers::Sabertooth2x5Controller;$<TARGET_FILE:sabertooth_2x5_controller>\n")

add_executable(sabertooth_2x5_controller_node
  src/sabertooth_2x5_controller_node.cpp
)
target_link_libraries(sabertooth_2x5_controller_node
sabertooth_2x5_controller
)

install(TARGETS
  servo_controller
  servo_controller_node
  sabertooth_2x5_controller
  sabertooth_2x5_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

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

ament_package()
