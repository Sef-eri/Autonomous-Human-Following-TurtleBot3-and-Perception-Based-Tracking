cmake_minimum_required(VERSION 3.8)
project(tb3_warehouse)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TargetVector.msg"
  "msg/GestureCmd.msg"
)

# Install Python executables directly
install(PROGRAMS
  scripts/vision_node.py
  scripts/control_node.py
  scripts/safety_node.py
  scripts/behavior_tree.py
  scripts/color_tracker_node.py
  scripts/follower_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch, worlds, models
install(DIRECTORY
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
