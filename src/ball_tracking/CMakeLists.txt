cmake_minimum_required(VERSION 3.5)
project(ball_tracking)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add include directories
include_directories(include)

# Add executable targets
add_executable(detect_ball_2D src/detect_ball_2D.cpp)
ament_target_dependencies(detect_ball_2D rclcpp std_msgs sensor_msgs cv_bridge image_transport)

add_executable(detect_ball_3D src/detect_ball_3D.cpp)
ament_target_dependencies(detect_ball_3D rclcpp std_msgs sensor_msgs cv_bridge image_transport)

add_executable(plan_path src/plan_path.cpp)
ament_target_dependencies(plan_path rclcpp std_msgs nav2_msgs nav_msgs geometry_msgs)

add_executable(follow_ball src/follow_ball.cpp)
ament_target_dependencies(follow_ball rclcpp std_msgs sensor_msgs cv_bridge image_transport nav2_common nav2_msgs nav_msgs geometry_msgs)

# Install targets
install(TARGETS
  detect_ball_2D
  detect_ball_3D
  follow_ball
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
