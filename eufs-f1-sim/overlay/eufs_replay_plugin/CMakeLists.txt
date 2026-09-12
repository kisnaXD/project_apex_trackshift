cmake_minimum_required(VERSION 3.5)
project(eufs_replay_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-inconsistent-missing-override)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

add_library(eufs_replay_plugin SHARED src/russell_replay_plugin.cpp)
target_include_directories(eufs_replay_plugin PUBLIC include)
ament_target_dependencies(eufs_replay_plugin gazebo_dev gazebo_ros nav_msgs rclcpp yaml_cpp_vendor)
target_link_libraries(eufs_replay_plugin yaml-cpp)

install(TARGETS eufs_replay_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/prepare_replay_sdf.py DESTINATION lib/${PROJECT_NAME})

ament_export_libraries(eufs_replay_plugin)
ament_export_dependencies(gazebo_dev gazebo_ros nav_msgs rclcpp yaml_cpp_vendor)
ament_package()
