#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("move_group");
#define PI 3.141592

#include "rclcpp/node_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/create_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <grasping_msgs/action/find_graspable_objects.hpp>

#include <thread>

using Find = grasping_msgs::action::FindGraspableObjects;
using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace MoveitScripts{
    class PickAndPlacePerception : public rclcpp::Node {
        public:
            PickAndPlacePerception(const rclcpp::NodeOptions & nodeOptions) : Node("get_pose_client", nodeOptions){
                this -> client_ptr_ = rclcpp_action::create_client<Find>(
                    this->get_node_base_interface(), 
                    this->get_node_graph_interface(),
                    this->get_node_logging_interface(),
                    this->get_node_waitables_interface(), 
                    "find_objects"
                );
                this -> timer_ = this -> create_wall_timer(500ms, [this](){
                    this->timer_->cancel();

                    this->goal_done_ = false;

                    if (!this->client_ptr_) {
                        RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
                    }

                    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
                        RCLCPP_ERROR(this->get_logger(),
                                "Action server not available after waiting");
                        this->goal_done_ = true;
                        return;
                    }

                    auto goal_msg = Find::Goal();
                    goal_msg.plan_grasps = false;

                    RCLCPP_INFO(this->get_logger(), "Sending goal");

                    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
                    send_goal_options.goal_response_callback =
                        std::bind(&PickAndPlacePerception::goal_response_callback, this, _1);
                    send_goal_options.feedback_callback =
                        std::bind(&PickAndPlacePerception::feedback_callback, this, _1, _2);
                    send_goal_options.result_callback =
                        std::bind(&PickAndPlacePerception::result_callback, this, _1);
                    auto goal_handle_future =
                        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
                });
            }


        protected:
            rclcpp_action::Client<Find>::SharedPtr client_ptr_;
            rclcpp::TimerBase::SharedPtr timer_;
            bool goal_done_;


            void goal_response_callback(const GoalHandleFind::SharedPtr &goal_handle) {
                if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                RCLCPP_INFO(this->get_logger(),
                            "Goal accepted by server, waiting for result");
                }
            }

            void feedback_callback(GoalHandleFind::SharedPtr,
                                    const std::shared_ptr<const Find::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Ignoring feedback...");
            }

            void result_callback(const GoalHandleFind::WrappedResult &result) {
                this->goal_done_ = true;
                switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                break;
                case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
                case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
                default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
                }

                RCLCPP_INFO(this->get_logger(), "Result received");
    
                
                RCLCPP_INFO(this->get_logger(), "X: %f",
                            result.result->objects[0].object.primitive_poses[0].position.x);
                RCLCPP_INFO(this->get_logger(), "Y: %f",
                            result.result->objects[0].object.primitive_poses[0].position.y);
      
                
                static const std::string PLANNING_GROUP_ARM = "manipulator_arm";

                rclcpp::NodeOptions node_options;
                node_options.automatically_declare_parameters_from_overrides(true);
                auto move_group_node =
                    rclcpp::Node::make_shared("move_group", node_options);

                rclcpp::executors::SingleThreadedExecutor executor;
                executor.add_node(move_group_node);
                std::thread([&executor]() { executor.spin(); }).detach();

                moveit::planning_interface::MoveGroupInterface move_group_arm(
                    move_group_node, PLANNING_GROUP_ARM);
            
                const moveit::core::JointModelGroup *joint_model_group_arm =
                    move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

                // // Get Current State
                moveit::core::RobotStatePtr current_state_arm =
                    move_group_arm.getCurrentState(10);

                std::vector<double> joint_group_positions_arm;
                current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                            joint_group_positions_arm);

                move_group_arm.setStartStateToCurrentState();

               

                //open the hand

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

                //  move hand to above object


                joint_group_positions_arm[0] = -25 * PI / 180;  // Shoulder Pan
                joint_group_positions_arm[1] = -83 * PI / 180; // Shoulder Lift
                joint_group_positions_arm[2] = 102 * PI / 180;  // Elbow
                joint_group_positions_arm[3] = -110 * PI / 180; // Wrist 1
                joint_group_positions_arm[4] = -91 * PI / 180; // Wrist 2
                joint_group_positions_arm[5] = 150 * PI / 180;  // Wrist 3

                move_group_arm.setJointValueTarget(joint_group_positions_arm);

                moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
                bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                                    moveit::core::MoveItErrorCode::SUCCESS);

                move_group_arm.execute(my_plan_arm);

                
                 // move hand to above object

                geometry_msgs::msg::Pose target_pose1;
                target_pose1.orientation.x = -1.0;
                target_pose1.orientation.y = 0.00;
                target_pose1.orientation.z = 0.00;
                target_pose1.orientation.w = 0.00;
                target_pose1.position.x = result.result->objects[0].object.primitive_poses[0].position.x + 0.01;
                target_pose1.position.y = result.result->objects[0].object.primitive_poses[0].position.y - 0.01;
                target_pose1.position.z = 0.21;
                
                // move_group_arm.setPoseTarget(target_pose1);
                // moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
                // bool success_arm = (move_group_arm.plan(my_plan_arm) ==
                //                     moveit::core::MoveItErrorCode::SUCCESS);

                // move_group_arm.execute(my_plan_arm);



                std::vector<geometry_msgs::msg::Pose> approach_waypoints;
                approach_waypoints.push_back(target_pose1);
                target_pose1.position.z -= 0.02;
                approach_waypoints.push_back(target_pose1);

                target_pose1.position.z -= 0.02;
                approach_waypoints.push_back(target_pose1);


                moveit_msgs::msg::RobotTrajectory trajectory_approach;
                const double jump_threshold = 0.0;
                const double eef_step = 0.01;

                double fraction = move_group_arm.computeCartesianPath(
                    approach_waypoints, eef_step, jump_threshold, trajectory_approach);

                move_group_arm.execute(trajectory_approach);

                current_state_hand -> copyJointGroupPositions(joint_model_group_hand, joint_group_positions_hand);

                joint_group_positions_hand[2] = 34 * PI / 180;

                move_group_hand.setJointValueTarget(joint_group_positions_hand);

                move_group_hand.plan(my_plan_hand);
                move_group_hand.execute(my_plan_hand);

                for(int i = (34 / 0.1); i < (35.5 / 0.1); i++) {

                     //fully close it
                    current_state_hand -> copyJointGroupPositions(joint_model_group_hand, joint_group_positions_hand);

                    joint_group_positions_hand[2] = (i * 0.1) * PI / 180;

                    move_group_hand.setJointValueTarget(joint_group_positions_hand);

                    move_group_hand.plan(my_plan_hand);
                    move_group_hand.execute(my_plan_hand);
                
                }
                

                //   move back up
                RCLCPP_INFO(LOGGER, "Approach to object!");

                std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
                target_pose1.position.z += 0.04;
                retreat_waypoints.push_back(target_pose1);

                target_pose1.position.z += 0.04;
                retreat_waypoints.push_back(target_pose1);

                target_pose1.position.z += 0.04;
                retreat_waypoints.push_back(target_pose1);

                

                moveit_msgs::msg::RobotTrajectory trajectory_retreat;
             

                fraction = move_group_arm.computeCartesianPath(
                    retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

                move_group_arm.execute(trajectory_retreat);

                // turn 180



                // // Get Current State
                current_state_arm =
                    move_group_arm.getCurrentState(10);

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
            }

    };

}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(MoveitScripts::PickAndPlacePerception)
