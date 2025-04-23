#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <string>
#include "weld_interfaces/action/validate.hpp"

using namespace std::chrono_literals;
using Validate = weld_interfaces::action::Validate;

class ValidateDummyServer : public rclcpp::Node
{
public:
  ValidateDummyServer()
  : Node("transit_dummy_server"), tick_(0), fail_joint_(false)
  {
    this->action_server_ = rclcpp_action::create_server<Validate>(
      this,
      "validate_joint",
      std::bind(&ValidateDummyServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ValidateDummyServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&ValidateDummyServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Validate Dummy Server is running.");
  }

private:
  rclcpp_action::Server<Validate>::SharedPtr action_server_;
  rclcpp::Time start_time_;
  bool fail_joint_; // Simulate a failure on joint6
  int tick_;
  const int duration_ = 2; // Duration in seconds

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Validate::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: joint=%s", goal->joint.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Validate>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Validate>> goal_handle)
  {
    std::thread{std::bind(&ValidateDummyServer::execute, this, goal_handle)}.detach();
  }

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<Validate>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing Validate...");
    start_time_ = this->now();
    auto feedback = std::make_shared<Validate::Feedback>();
    feedback->completion = 0.0;

    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Validating %s", goal->joint.c_str());

    while (rclcpp::ok()) {
      auto elapsed_time = this->now() - start_time_;
      if (goal_handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          auto result = std::make_shared<Validate::Result>();
          result->success = false;
          goal_handle->canceled(result);
          tick_ = 0; // Reset tick counter
          return;
      }

      if (elapsed_time.seconds() >= duration_) {
        break;
      }

      if (fail_joint_ && goal->joint == "joint6") {
        RCLCPP_ERROR(this->get_logger(), "Validate failed on joint: %s", goal->joint.c_str());
        fail_joint_ = false; // Reset fail_joint_ to prevent repeated failure
        auto result = std::make_shared<Validate::Result>();
        result->success = false;
        goal_handle->abort(result);
        tick_ = 0; // Reset tick counter
        return;
      }

      feedback->completion = elapsed_time.seconds() / static_cast<double>(duration_);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Validate tick: %d", ++tick_);

      std::this_thread::sleep_for(200ms);
    }

    if (rclcpp::ok()) {
      auto result = std::make_shared<Validate::Result>();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Validate completed successfully.");
    }

    tick_ = 0; // Reset tick counter
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ValidateDummyServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}