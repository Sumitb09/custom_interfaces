cmake_minimum_required(VERSION 3.5)
project(custom_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

# Define action files
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Wait.action"
)

ament_export_dependencies(rosidl_default_runtime)

install(
  PROGRAMS
  src/task_server.py
  src/task_client.py
  src/task_publisher.py
  DESTINATION lib/${PROJECT_NAME}

  
)

ament_package()
