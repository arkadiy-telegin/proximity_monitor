cmake_minimum_required(VERSION 3.5)
project(your_package_name)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/ProximityWarning.msg"
# )
# We use built-in messages to avoid the need to compile for Unity

ament_package()
