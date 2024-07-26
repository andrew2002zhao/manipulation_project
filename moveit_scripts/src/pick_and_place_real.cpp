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


  joint_group_positions_arm[0] = -6 * PI / 180;  // Shoulder Pan
  joint_group_positions_arm[1] = -51 * PI / 180; // Shoulder Lift
  joint_group_positions_arm[2] = 3 * PI / 180;  // Elbow
  joint_group_positions_arm[3] = -43 * PI / 180; // Wrist 1
  joint_group_positions_arm[4] = -91 * PI / 180; // Wrist 2
  joint_group_positions_arm[5] = 169 * PI / 180;  // Wrist 3

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                      moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);

 

// //   //open the hand

  static const std::string PLANNING_GROUP_HAND = "manipulator_hand";
  moveit::planning_interface::MoveGroupInterface move_group_hand(
    move_group_node, PLANNING_GROUP_HAND
  );

  
  const moveit::core::JointModelGroup *joint_model_group_hand =
      move_group_hand.getCurrentState()->getJointModelGroup(PLANNING_GROUP_HAND);
  moveit::core::RobotStatePtr current_state_hand = 
    move_group_hand.getCurrentState(10);
  std::vector<double> joint_group_positions_hand;
  current_state_hand -> copyJointGroupPositions(joint_model_group_hand, joint_group_positions_hand);

  joint_group_positions_hand[2] = 0;

  move_group_hand.setJointValueTarget(joint_group_positions_hand);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_hand;
  move_group_hand.plan(my_plan_hand);
  move_group_hand.execute(my_plan_hand);

  


  //lower arm by 5cm
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;

  // move hand to above object

  geometry_msgs::msg::Pose target_pose1;
  target_pose1.orientation.x = -1.0;
  target_pose1.orientation.y = 0.00;
  target_pose1.orientation.z = 0.00;
  target_pose1.orientation.w = 0.00;
  target_pose1.position.x = 0.343;
  target_pose1.position.y = 0.132;
  target_pose1.position.z = 0.264;
 
  approach_waypoints.push_back(target_pose1);
  

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);

  target_pose1.position.z -= 0.03;
  approach_waypoints.push_back(target_pose1);


  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);
  move_group_arm.execute(trajectory_approach);

//   //close hand
//     //approach the hand
  current_state_hand -> copyJointGroupPositions(joint_model_group_hand, joint_group_positions_hand);

  joint_group_positions_hand[2] = 30 * PI / 180;

  move_group_hand.setJointValueTarget(joint_group_positions_hand);

  move_group_hand.plan(my_plan_hand);
  move_group_hand.execute(my_plan_hand);
//     //fully close it
  current_state_hand -> copyJointGroupPositions(joint_model_group_hand, joint_group_positions_hand);

  joint_group_positions_hand[2] = 37.1 * PI / 180;

  move_group_hand.setJointValueTarget(joint_group_positions_hand);

  move_group_hand.plan(my_plan_hand);
  move_group_hand.execute(my_plan_hand);

// // //   move back up
//      //lower arm by 5cm

  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
  target_pose1.position.z += 0.05;
  retreat_waypoints.push_back(target_pose1);

  target_pose1.position.z += 0.05;
  retreat_waypoints.push_back(target_pose1);


  moveit_msgs::msg::RobotTrajectory retreat_approach;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, retreat_approach);
  move_group_arm.execute(retreat_approach);


// //   // turn 180

    joint_model_group_arm = move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

    current_state_arm = move_group_arm.getCurrentState(10);
    
    current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                joint_group_positions_arm);

    joint_group_positions_arm[0] = joint_group_positions_arm[0] + PI;  // Shoulder Pan
    move_group_arm.setJointValueTarget(joint_group_positions_arm);

    success_arm = (move_group_arm.plan(my_plan_arm) ==
                    moveit::core::MoveItErrorCode::SUCCESS);

    move_group_arm.execute(my_plan_arm);

    //open the hand

    current_state_hand -> copyJointGroupPositions(joint_model_group_hand, joint_group_positions_hand);

    joint_group_positions_hand[2] = 0;

    move_group_hand.setJointValueTarget(joint_group_positions_hand);

    move_group_hand.plan(my_plan_hand);
    move_group_hand.execute(my_plan_hand);


  rclcpp::shutdown();
  return 0;
}