cmake_minimum_required(VERSION 3.5)
project(ssc32u_driver)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ssc32u_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(ssc32u_driver SHARED
  src/ssc32u_driver.cpp)
target_compile_definitions(ssc32u_driver
  PRIVATE "SSC32U_BUILDING_DLL")
ament_target_dependencies(ssc32u_driver
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "lifecycle_msgs"
  "std_msgs"
  "ssc32u_msgs")
rclcpp_components_register_nodes(ssc32u_driver "ssc32u_driver::SSC32UDriver")
set(node_plugins "${node_plugins}ssc32u_driver::SSC32UDriver;$<TARGET_FILE:ssc32u_driver>\n")

install(TARGETS
  ssc32u_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

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
