#include "jigless-planner/ExecuteBottom.hpp"
#include <rclcpp_action/client.hpp>

using namespace std::chrono_literals;

ExecuteBottom::ExecuteBottom() : plansys2::ActionExecutorClient("execute", 100ms)
{
  RCLCPP_INFO(this->get_logger(), "Initializing bottom executor action");
  action_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  publisher_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto qos_setting = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_setting.reliable();
  publisher_opts_.callback_group = publisher_group_;
  joint_status_publisher_ = this->create_publisher<jigless_planner_interfaces::msg::JointStatus>(
    "failed_joints", qos_setting);
  problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
  this->declare_parameter("bottom_ns", rclcpp::PARAMETER_STRING);

}

void ExecuteBottom::do_work() {}

std::vector<std::string> ExecuteBottom::getCommandedJoints()
{
  std::vector<std::string> commanded_joints;
  std::vector<plansys2::Predicate> predicates = problem_expert_->getPredicates();
  for (std::size_t i = 0; i < predicates.size(); ++i) {
    if (predicates[i].name == "commanded")
      commanded_joints.emplace_back(std::move(predicates[i].parameters[0].name));
  }
  return commanded_joints;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  ExecuteBottom::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Activating bottom executor action");
  send_feedback(0.0, "Execute Bottom starting");
  std::string bottom_ns = this->get_parameter("bottom_ns").as_string();
  std::string topic = "/" + bottom_ns + "/run_bottom";
  RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic.c_str());
  action_client_ = rclcpp_action::create_client<RunBottom>(
    shared_from_this(), "/" + bottom_ns + "/run_bottom");
  bool bottom_controller_ready = false;
  do
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for action server...");
    bottom_controller_ready = action_client_->wait_for_action_server(std::chrono::seconds(5));
  } while (bottom_controller_ready);
  RCLCPP_INFO(this->get_logger(), "Bottom controller action server ready");
  jigless_planner_interfaces::action::RunBottom::Goal goal;
  goal.joints.joints = std::move(getCommandedJoints());
  goal.operation = goal.START;
  auto send_goal_options = rclcpp_action::Client<RunBottom>::SendGoalOptions();
  send_goal_options.feedback_callback = [this](
    GoalHandleRunBottom::SharedPtr,
    const std::shared_ptr<const RunBottom::Feedback> & feedback) {
    RCLCPP_INFO(this->get_logger(), "Execute Bottom running");
    int total_goals = feedback->action_execution_status.size();
    int completed_goals = 0;
    for (const auto &action : feedback->action_execution_status) {
      if (action.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
        RCLCPP_INFO(this->get_logger(), "Executing action: %s with progress: %f",
        action.action_full_name.c_str(), action.completion);
      }
      if (action.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED)
        ++completed_goals;
    }
  send_feedback(completed_goals / total_goals, "Execute Bottom running");
    };
  send_goal_options.result_callback = [this](
    const GoalHandleRunBottom::WrappedResult & result) {
      bool finished = result.code == rclcpp_action::ResultCode::SUCCEEDED, publish_msg_ = false;
      auto message = jigless_planner_interfaces::msg::JointStatus();
      message.joints.reserve(result.result->failed_joints.joints.size());
      message.status.reserve(result.result->failed_joints.joints.size());
      for (std::size_t i = 0; i < result.result->failed_joints.joints.size(); ++i) {
        message.joints.emplace_back(std::move(result.result->failed_joints.joints[i]));
        message.status.emplace_back(std::move(result.result->failed_joints.status[i]));
        if(!publish_msg_)
          publish_msg_ = result.result->failed_joints.status[i];
      }
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Execute Bottom completed");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(this->get_logger(), "Execute Bottom aborted");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(this->get_logger(), "Execute Bottom canceled");
          break;
      }
      if (publish_msg_) {
        joint_status_publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Publishing failed joint status");
      }
      finish(finished, 1.0, "Execute Bottom completed");
    };
  RCLCPP_INFO(this->get_logger(), "Sending goal to bottom controller");
  future_executor_goal_handle_ = action_client_->async_send_goal(goal, send_goal_options);
  return ActionExecutorClient::on_activate(previous_state);
}

void ExecuteBottom::feedbackCallback(
  GoalHandleRunBottom::SharedPtr,
  const std::shared_ptr<const RunBottom::Feedback> & feedback)
{
  RCLCPP_INFO(this->get_logger(), "Execute Bottom running");
  int total_goals = feedback->action_execution_status.size();
  int completed_goals = 0;
  for (const auto &action : feedback->action_execution_status) {
    if (action.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
      RCLCPP_INFO(this->get_logger(), "Executing action: %s with progress: %f",
      action.action_full_name.c_str(), action.completion);
    }
    if (action.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED)
      ++completed_goals;
  }
  send_feedback(completed_goals / total_goals, "Execute Bottom running");
}

void ExecuteBottom::resultCallback(const GoalHandleRunBottom::WrappedResult & result)
{
  bool finished = result.code == rclcpp_action::ResultCode::SUCCEEDED, publish_msg_ = false;
  auto message = jigless_planner_interfaces::msg::JointStatus();
  message.joints.reserve(result.result->failed_joints.joints.size());
  message.status.reserve(result.result->failed_joints.joints.size());
  for (std::size_t i = 0; i < result.result->failed_joints.joints.size(); ++i) {
    message.joints.emplace_back(std::move(result.result->failed_joints.joints[i]));
    message.status.emplace_back(std::move(result.result->failed_joints.status[i]));
    if(!publish_msg_)
      publish_msg_ = result.result->failed_joints.status[i];
  }
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Execute Bottom completed");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Execute Bottom aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Execute Bottom canceled");
      break;
  }
  if (publish_msg_) {
    joint_status_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing failed joint status");
  }
  finish(finished, 1.0, "Execute Bottom completed");
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExecuteBottom>();
  node->set_parameter(rclcpp::Parameter("action_name", "execute"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}