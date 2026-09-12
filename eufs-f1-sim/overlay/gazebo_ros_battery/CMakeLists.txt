cmake_minimum_required(VERSION 3.8)
project(gazebo_ros_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_consumer_base SHARED src/battery_consumer_base.cpp)
target_link_libraries(${PROJECT_NAME}_consumer_base ${GAZEBO_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_consumer_base gazebo_ros rclcpp)

add_library(${PROJECT_NAME}_discharge SHARED src/battery_discharge.cpp)
target_link_libraries(${PROJECT_NAME}_discharge ${GAZEBO_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_discharge gazebo_ros rclcpp sensor_msgs std_msgs)

add_library(gazebo_ros_mechanical_energy_battery_consumer SHARED src/mechanical_energy_consumer.cpp)
target_link_libraries(gazebo_ros_mechanical_energy_battery_consumer ${PROJECT_NAME}_consumer_base ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_ros_mechanical_energy_battery_consumer gazebo_ros rclcpp)

add_library(gazebo_ros_energy_aware_ackermann_gate SHARED src/energy_aware_ackermann_gate.cpp)
target_link_libraries(gazebo_ros_energy_aware_ackermann_gate ${GAZEBO_LIBRARIES})
ament_target_dependencies(gazebo_ros_energy_aware_ackermann_gate gazebo_ros geometry_msgs rclcpp sensor_msgs std_msgs)

install(TARGETS
  ${PROJECT_NAME}_consumer_base
  ${PROJECT_NAME}_discharge
  gazebo_ros_mechanical_energy_battery_consumer
  gazebo_ros_energy_aware_ackermann_gate
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
