cmake_minimum_required(VERSION 3.8)
project(turtlesimAutomata)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
#find_package(turtlesimAutomata REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces)


#rosidl_generate_interfaces(${PROJECT_NAME}
#  "srv/acknowledge.srv"
#  DEPENDENCIES std_msgs
#)

# Include directories
include_directories(include)



add_executable(edge src/edge.cpp)
ament_target_dependencies(edge rclcpp std_msgs)
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)
#target_compile_features(edge PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(turn_turtle src/turn_turtle.cpp)
ament_target_dependencies(turn_turtle rclcpp std_msgs turtlesim geometry_msgs)

add_executable(drive_turtle_continuously src/drive_turtle_continuously.cpp)
ament_target_dependencies(drive_turtle_continuously std_msgs rclcpp geometry_msgs)

add_executable(start_turtle src/start_turtle.cpp)
ament_target_dependencies(start_turtle std_msgs rclcpp rcl_interfaces)
target_include_directories(turn_turtle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  
  $<INSTALL_INTERFACE:include>)
target_compile_features(turn_turtle PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS
  edge
  turn_turtle
  drive_turtle_continuously
  start_turtle
  
  DESTINATION lib/${PROJECT_NAME})

  
#this is important for the launchfile to be accesible from the ros2_ws
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
