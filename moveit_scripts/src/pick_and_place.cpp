#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");
#define PI 3.141592

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("move_group", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "manipulator_arm";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  move_group_arm.setStartStateToCurrentState();

  // Go Home
  RCLCPP_INFO(LOGGER, "Going Home");

  joint_group_positions_arm[0] = -15 * PI / 180;  // Shoulder Pan
  joint_group_positions_arm[1] = -90 * PI / 180; // Shoulder Lift
  joint_group_positions_arm[2] = 77 * PI / 180;  // Elbow
  joint_group_positions_arm[3] = -89 * PI / 180; // Wrist 1
  joint_group_positions_arm[4] = -100 * PI / 180; // Wrist 2
  joint_group_positions_arm[5] = 0;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

//   // Pregrasp
//   RCLCPP_INFO(LOGGER, "Pregrasp Position");

//   geometry_msgs::msg::Pose target_pose1;
//   target_pose1.orientation.x = -1.0;
//   target_pose1.orientation.y = 0.00;
//   target_pose1.orientation.z = 0.00;
//   target_pose1.orientation.w = 0.00;
//   target_pose1.position.x = 0.343;
//   target_pose1.position.y = 0.132;
//   target_pose1.position.z = 0.264;
//   move_group_arm.setPoseTarget(target_pose1);

//   success_arm = (move_group_arm.plan(my_plan_arm) ==
//                  moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_arm.execute(my_plan_arm);

//   // Approach
//   RCLCPP_INFO(LOGGER, "Approach to object!");

//   std::vector<geometry_msgs::msg::Pose> approach_waypoints;
//   target_pose1.position.z -= 0.03;
//   approach_waypoints.push_back(target_pose1);

//   target_pose1.position.z -= 0.03;
//   approach_waypoints.push_back(target_pose1);

//   moveit_msgs::msg::RobotTrajectory trajectory_approach;
//   const double jump_threshold = 0.0;
//   const double eef_step = 0.01;

//   double fraction = move_group_arm.computeCartesianPath(
//       approach_waypoints, eef_step, jump_threshold, trajectory_approach);

//   move_group_arm.execute(trajectory_approach);

//   // Retreat

//   RCLCPP_INFO(LOGGER, "Retreat from object!");

//   std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
//   target_pose1.position.z += 0.03;
//   retreat_waypoints.push_back(target_pose1);

//   target_pose1.position.z += 0.03;
//   retreat_waypoints.push_back(target_pose1);

//   moveit_msgs::msg::RobotTrajectory trajectory_retreat;

//   fraction = move_group_arm.computeCartesianPath(
//       retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

//   move_group_arm.execute(trajectory_retreat);

  rclcpp::shutdown();
  return 0;
}