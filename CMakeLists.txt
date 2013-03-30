cmake_minimum_required(VERSION 2.8.3)
project(lynxmotion_ssc32)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs trajectory_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ssc32_driver ssc32
   CATKIN_DEPENDS roscpp
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(ssc32 src/ssc32.cpp)
target_link_libraries(ssc32 ${catkin_LIBRARIES})

add_library(ssc32_driver src/ssc32_driver.cpp)
target_link_libraries(ssc32_driver ssc32 ${catkin_LIBRARIES})

add_executable(ssc32_node src/ssc32_node.cpp)
target_link_libraries(ssc32_node ssc32_driver ${catkin_LIBRARIES})

add_executable(test_ssc32 src/test_ssc32.cpp)
target_link_libraries(test_ssc32 ssc32_driver ${catkin_LIBRARIES})

install(TARGETS ssc32 ssc32_driver ssc32_node test_ssc32
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
