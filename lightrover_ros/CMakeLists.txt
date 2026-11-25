cmake_minimum_required(VERSION 3.8)
project(lightrover_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(lightrover_interface REQUIRED)

ament_python_install_package(${PROJECT_NAME})

include_directories(include)

add_library(vs_wrc201_motor_cpp src/vs_wrc201_motor_cpp.cpp)
target_include_directories(vs_wrc201_motor_cpp PUBLIC include)
ament_target_dependencies(vs_wrc201_motor_cpp rclcpp)

add_executable(rover_gamepad_cpp src/rover_gamepad_cpp.cpp)
ament_target_dependencies(rover_gamepad_cpp rclcpp geometry_msgs sensor_msgs)

add_executable(i2c_controller_cpp src/i2c_controller_cpp.cpp)
ament_target_dependencies(i2c_controller_cpp rclcpp lightrover_interface)

add_executable(pos_controller_cpp src/pos_controller_cpp.cpp)
ament_target_dependencies(pos_controller_cpp rclcpp geometry_msgs nav_msgs lightrover_interface)
target_link_libraries(pos_controller_cpp vs_wrc201_motor_cpp)

add_executable(odom_manager_cpp src/odometry_cpp.cpp)
ament_target_dependencies(odom_manager_cpp rclcpp geometry_msgs nav_msgs tf2_ros lightrover_interface)

add_executable(set_initial_pose_cpp src/set_initial_pose_cpp.cpp)
ament_target_dependencies(set_initial_pose_cpp rclcpp geometry_msgs nav2_msgs)

add_executable(send_nav_goal_cpp src/send_nav_goal_cpp.cpp)
ament_target_dependencies(send_nav_goal_cpp rclcpp rclcpp_action geometry_msgs nav2_msgs)

install(TARGETS
  i2c_controller_cpp
  rover_gamepad_cpp
  pos_controller_cpp
  odom_manager_cpp
  set_initial_pose_cpp
  send_nav_goal_cpp
  DESTINATION lib/${PROJECT_NAME})

function(install_python_executable source_file executable_name)
  install(PROGRAMS
    ${source_file}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${executable_name})
endfunction()

install_python_executable(lightrover_ros/i2c_controller.py i2c_controller)
install_python_executable(lightrover_ros/odometry.py odom_manager)
install_python_executable(lightrover_ros/pos_controller.py pos_controller)
install_python_executable(lightrover_ros/rover_gamepad.py rover_gamepad)
install_python_executable(lightrover_ros/rover_keyboard.py rover_keyboard)
install_python_executable(lightrover_ros/rover_mouse.py rover_mouse)
install_python_executable(lightrover_ros/set_initial_pose.py set_initial_pose)
install_python_executable(lightrover_ros/send_nav_goal.py send_nav_goal)
install_python_executable(lightrover_ros/ydlidar_driver_probe.py ydlidar_driver_probe)
install_python_executable(lightrover_ros/ydlidar_serial_probe.py ydlidar_serial_probe)

install(DIRECTORY launch params config configuration_files
  DESTINATION share/${PROJECT_NAME})

ament_package()
