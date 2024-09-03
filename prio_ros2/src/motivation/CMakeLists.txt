cmake_minimum_required(VERSION 3.8)
project(motivation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
add_executable(talker src/talker.cpp)
add_executable(intra_talker src/intra_talker.cpp)
add_executable(listener src/listener.cpp)
add_executable(trace_first src/trace_first.cpp)
add_executable(trace_medium src/trace_medium.cpp)
add_executable(trace_last src/trace_last.cpp)
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(intra_talker rclcpp std_msgs)
ament_target_dependencies(listener rclcpp std_msgs)
ament_target_dependencies(trace_first rclcpp std_msgs)
ament_target_dependencies(trace_medium rclcpp std_msgs)
ament_target_dependencies(trace_last rclcpp std_msgs)

install(TARGETS
	talker
	intra_talker
	listener
	trace_first
        trace_medium
        trace_last
	DESTINATION lib/${PROJECT_NAME})

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
