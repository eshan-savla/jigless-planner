#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <string>
#include "weld_interfaces/action/weld.hpp"
#include "jigless_planner_interfaces/srv/interact_top.hpp"

using namespace std::chrono_literals;
using Weld = weld_interfaces::action::Weld;

class WeldDummyServer : public rclcpp::Node
{
public:
  WeldDummyServer()
  : Node("transit_dummy_server"), tick_(0), fail_joint_(false)
  {
    this->action_server_ = rclcpp_action::create_server<Weld>(
      this,
      "weld_joint",
      std::bind(&WeldDummyServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WeldDummyServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&WeldDummyServer::handle_accepted, this, std::placeholders::_1)
    );
    service_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->client_ = this->create_client<jigless_planner_interfaces::srv::InteractTop>(
        "/top_planner/interact_top", rmw_qos_profile_services_default, service_group);
    RCLCPP_INFO(this->get_logger(), "Weld Dummy Server is running.");
  }

private:
  rclcpp_action::Server<Weld>::SharedPtr action_server_;
  rclcpp::Client<jigless_planner_interfaces::srv::InteractTop>::SharedPtr client_;
  rclcpp::CallbackGroup::SharedPtr service_group;
  rclcpp::Time start_time_;
  bool fail_joint_;
  int tick_;
  const int duration_ = 5; // Duration in seconds

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Weld::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: joint=%s", goal->joint.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Weld>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Weld>> goal_handle)
  {
    std::thread{std::bind(&WeldDummyServer::execute, this, goal_handle)}.detach();
  }

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Weld>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing Weld...");
    start_time_ = this->now();
    auto feedback = std::make_shared<Weld::Feedback>();
    feedback->completion = 0.0;

    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Welding %s", goal->joint.c_str());

    while (rclcpp::ok()) {
      auto elapsed_time = this->now() - start_time_;
      if (goal_handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          auto result = std::make_shared<Weld::Result>();
          result->success = false;
          goal_handle->canceled(result);
          tick_ = 0; // Reset tick counter
          return;
      }

      if (elapsed_time.seconds() >= duration_) {
        break;
      }

      if (tick_ >= 4 && fail_joint_ && goal->joint == "joint6") {
        RCLCPP_ERROR(this->get_logger(), "Weld failed on joint: %s", goal->joint.c_str());
        fail_joint_ = false; // Reset fail_joint_ to prevent repeated failure
        while (!client_->wait_for_service(1s)) {
          RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }
        auto request = std::make_shared<jigless_planner_interfaces::srv::InteractTop::Request>();
        request->operation = jigless_planner_interfaces::srv::InteractTop::Request::STOP;
        auto future = client_->async_send_request(request).future.share();
        auto status = future.wait_for(3s);
        if (status == std::future_status::ready) {
          auto result = future.get();
          if (result->success) {
            RCLCPP_INFO(this->get_logger(), "Service call succeeded");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
          }
        } else {
          RCLCPP_ERROR(this->get_logger(), "Service call timed out");
        }
        auto result = std::make_shared<Weld::Result>();
        result->success = false;
        goal_handle->abort(result);
        tick_ = 0; // Reset tick counter
        return;
      }
      feedback->completion = elapsed_time.seconds() / static_cast<double>(duration_);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Weld tick: %d", ++tick_);

      std::this_thread::sleep_for(200ms);
    }

    if (rclcpp::ok()) {
      auto result = std::make_shared<Weld::Result>();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Weld completed successfully.");
    }

    tick_ = 0; // Reset tick counter
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WeldDummyServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}