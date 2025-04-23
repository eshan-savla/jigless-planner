#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <memory>
#include <string>
#include "transit_interfaces/action/generate_transit.hpp"

using namespace std::chrono_literals;
using GenerateTransit = transit_interfaces::action::GenerateTransit;

class TransitDummyServer : public rclcpp::Node
{
public:
  TransitDummyServer()
  : Node("transit_dummy_server"), tick_(0), fail_joint_(false)
  {
    this->action_server_ = rclcpp_action::create_server<GenerateTransit>(
      this,
      "generate_transit",
      std::bind(&TransitDummyServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TransitDummyServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&TransitDummyServer::handle_accepted, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Transit Dummy Server is running.");
  }

private:
  rclcpp_action::Server<GenerateTransit>::SharedPtr action_server_;
  rclcpp::Time start_time_;
  bool fail_joint_;
  int tick_;
  const int duration_ = 2; // Duration in seconds

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GenerateTransit::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request: from_joint=%s, to_joint=%s",
                goal->from_joint.c_str(), goal->to_joint.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GenerateTransit>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GenerateTransit>> goal_handle)
  {
    std::thread{std::bind(&TransitDummyServer::execute, this, goal_handle)}.detach();
  }

  void execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<GenerateTransit>> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing Transit...");
    start_time_ = this->now();
    auto feedback = std::make_shared<GenerateTransit::Feedback>();
    feedback->completion = 0.0;

    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Transiting from %s to %s", goal->from_joint.c_str(), goal->to_joint.c_str());
    std::stringstream ss;
    ss << "Moving workpieces: ";
    for (size_t i = 0; i < goal->workpiece1.size(); ++i) {
      ss << goal->workpiece1[i];
      if (i != goal->workpiece1.size() - 1) {
        ss << " + ";
      }
      else
        ss << std::endl;
    }
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    if (!goal->workpiece2.empty()) {
      ss.str("");
      ss << "Moving workpieces: ";
      for (size_t i = 0; i < goal->workpiece2.size(); ++i) {
        ss << goal->workpiece2[i];
        if (i != goal->workpiece2.size() - 1) {
          ss << " + ";
        }
        else
          ss << std::endl;
      }
      RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
    }

    while (rclcpp::ok()) {
      auto elapsed_time = this->now() - start_time_;
      if (goal_handle->is_canceling()) {
          RCLCPP_INFO(this->get_logger(), "Goal canceled");
          auto result = std::make_shared<GenerateTransit::Result>();
          result->success = false;
          goal_handle->canceled(result);
          tick_ = 0; // Reset tick counter
          return;
      }

      if (elapsed_time.seconds() >= duration_) {
        break;
      }

      if (fail_joint_ && goal->to_joint == "joint6") {
        RCLCPP_ERROR(this->get_logger(), "Transit failed to joint: %s", goal->to_joint.c_str());
        fail_joint_ = false; // Reset fail_joint_ to prevent repeated failure
        auto result = std::make_shared<GenerateTransit::Result>();
        result->success = false;
        goal_handle->abort(result);
        tick_ = 0; // Reset tick counter
        return;
      }

      feedback->completion = elapsed_time.seconds() / static_cast<double>(duration_);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Transit tick: %d", ++tick_);

      std::this_thread::sleep_for(200ms);
    }

    if (rclcpp::ok()) {
      auto result = std::make_shared<GenerateTransit::Result>();
      result->success = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Transit completed successfully.");
    }

    tick_ = 0; // Reset tick counter
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TransitDummyServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}