cmake_minimum_required(VERSION 3.8)
project(package_121355690)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(control_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(package_121355690 src/package_121355690.cpp)

ament_target_dependencies(package_121355690 
  rclcpp 
  rclcpp_action 
  moveit_ros_planning_interface 
  control_msgs
  shape_msgs
  geometry_msgs
)

target_include_directories(package_121355690 PUBLIC 
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> 
  $<INSTALL_INTERFACE:include>
)

target_compile_features(package_121355690 PUBLIC c_std_99 cxx_std_17)

install(TARGETS package_121355690 DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
