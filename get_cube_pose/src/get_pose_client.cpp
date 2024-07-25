#include "rclcpp/node_options.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_action/create_client.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <grasping_msgs/action/find_graspable_objects.hpp>

using Find = grasping_msgs::action::FindGraspableObjects;
using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace GetCubePose{
    class GetPoseClient : public rclcpp::Node {
        public:
            GetPoseClient(const rclcpp::NodeOptions & nodeOptions) : Node("get_pose_client", nodeOptions){
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
                        std::bind(&GetPoseClient::goal_response_callback, this, _1);
                    send_goal_options.feedback_callback =
                        std::bind(&GetPoseClient::feedback_callback, this, _1, _2);
                    send_goal_options.result_callback =
                        std::bind(&GetPoseClient::result_callback, this, _1);
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
                //}
            }

    };

}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(GetCubePose::GetPoseClient)
