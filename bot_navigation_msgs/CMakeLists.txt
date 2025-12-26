cmake_minimum_required(VERSION 3.8)
project(bot_navigation_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CreateTask.srv"
  "srv/TaskControl.srv"
  "srv/GetTaskStatus.srv"
  "srv/ListTasks.srv"
  "srv/StartExploration.srv"
  "srv/StartPatrol.srv"
  "srv/WaypointControl.srv"
  "srv/RecordWaypoints.srv"
  "srv/NavigateToPose.srv"
  "srv/EmergencyStop.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
