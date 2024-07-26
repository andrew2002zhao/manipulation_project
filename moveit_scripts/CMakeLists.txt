cmake_minimum_required(VERSION 3.8)
project(moveit_scripts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(grasping_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

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

add_executable(pick_and_place src/pick_and_place.cpp)
target_include_directories(pick_and_place PUBLIC include)

ament_target_dependencies(pick_and_place
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	interactive_markers
	geometric_shapes
	control_msgs
	moveit_msgs
)

add_executable(pick_and_place_real src/pick_and_place_real.cpp)
target_include_directories(pick_and_place_real PUBLIC include)

ament_target_dependencies(pick_and_place_real
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	interactive_markers
	geometric_shapes
	control_msgs
	moveit_msgs
)

add_library(pick_and_place_perception SHARED src/pick_and_place_perception.cpp)
ament_target_dependencies(pick_and_place_perception
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	interactive_markers
	geometric_shapes
	control_msgs
	moveit_msgs
	grasping_msgs
	rclcpp_components
)
RCLCPP_COMPONENTS_REGISTER_NODES(pick_and_place_perception "MoveitScripts::PickAndPlacePerception")

add_library(pick_and_place_perception_real SHARED src/pick_and_place_perception_real.cpp)
ament_target_dependencies(pick_and_place_perception_real
	rclcpp
	rclcpp_action
	moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
	interactive_markers
	geometric_shapes
	control_msgs
	moveit_msgs
	grasping_msgs
	rclcpp_components
)
RCLCPP_COMPONENTS_REGISTER_NODES(pick_and_place_perception_real "MoveitScripts::PickAndPlacePerceptionReal")

install(TARGETS
	pick_and_place_perception
	pick_and_place_perception_real
	DESTINATION 
	lib/
)

install(TARGETS 
	pick_and_place
	pick_and_place_real
	
    DESTINATION lib/${PROJECT_NAME}
)

# Install the launch file
install(DIRECTORY 
	launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
