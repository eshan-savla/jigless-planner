#include "jigless-planner/ExecuteBottom.hpp"
#include <rclcpp_action/client.hpp>

using namespace std::chrono_literals;

ExecuteBottom::ExecuteBottom() : plansys2::ActionExecutorClient("execute", 100ms)
{
  RCLCPP_INFO(this->get_logger(), "Initializing bottom executor action");
  action_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  publisher_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto qos_setting = rclcpp::QoS(rclcpp::KeepLast(1));
  qos_setting.reliable();
  publisher_opts_.callback_group = publisher_group_;
  joint_status_publisher_ = this->create_publisher<jigless_planner_interfaces::msg::JointStatus>(
    "failed_joints", qos_setting);
  execute_started_publisher_ = this->create_publisher<std_msgs::msg::Empty>(
    "execute_bottom_started", qos_setting);
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
    if (predicates[i].name == "welded") {
      auto it = std::find(commanded_joints.begin(), commanded_joints.end(),
        predicates[i].parameters[0].name);
      if (it != commanded_joints.end()) {
        commanded_joints.erase(it);
      }
    }
  }
  return commanded_joints;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  ExecuteBottom::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(this->get_logger(), "Activating bottom executor action");
  std::string bottom_ns = this->get_parameter("bottom_ns").as_string();
  std::string topic = "/" + bottom_ns + "/run_bottom";
  RCLCPP_INFO(this->get_logger(), "Topic name: %s", topic.c_str());
  action_client_ = rclcpp_action::create_client<RunBottom>(
    shared_from_this(), topic);
  bool bottom_controller_ready = false;
  do
  {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for action server...");
    bottom_controller_ready = action_client_->wait_for_action_server(std::chrono::seconds(5));
  } while (!bottom_controller_ready);
  RCLCPP_INFO(this->get_logger(), "Bottom controller action server ready");
  jigless_planner_interfaces::action::RunBottom::Goal goal;
  goal.joints.joints = std::move(getCommandedJoints());
  goal.operation = goal.START;
  auto send_goal_options = rclcpp_action::Client<RunBottom>::SendGoalOptions();
  send_goal_options.feedback_callback = std::bind(
    &ExecuteBottom::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(
    &ExecuteBottom::resultCallback, this, std::placeholders::_1);
  send_goal_options.goal_response_callback = std::bind(
    &ExecuteBottom::responseCallback, this, std::placeholders::_1);
  RCLCPP_INFO(this->get_logger(), "Sending goal to bottom controller");
  future_executor_goal_handle_ = action_client_->async_send_goal(goal, send_goal_options);
  finished_ = false;
  return ActionExecutorClient::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  ExecuteBottom::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  if (action_client_ && !finished_){
    RCLCPP_INFO(this->get_logger(), "Cancelling bottom controller execution");
    auto goal_handle = future_executor_goal_handle_.get();
    if (goal_handle) {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Goal handle is null");
      auto message = jigless_planner_interfaces::msg::JointStatus();
      joint_status_publisher_->publish(message);
    }
  }
  return ActionExecutorClient::on_deactivate(previous_state);
}

void ExecuteBottom::responseCallback(const std::shared_ptr<GoalHandleRunBottom> &response)
{
  if (response) {
    RCLCPP_INFO(this->get_logger(), "Goal accepted");
    status_ = "expect_joints";
    accepted_ = true;
    auto message = std_msgs::msg::Empty();
    execute_started_publisher_->publish(message);
  } else {
    status_ = "Failed to execute";
    finished_ = true;
  }
  send_feedback(0, status_);
}

void
ExecuteBottom::feedbackCallback(
    GoalHandleRunBottom::SharedPtr,
    const std::shared_ptr<const RunBottom::Feedback> &feedback)
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Execute Bottom running");
  RCLCPP_WARN_EXPRESSION(this->get_logger(), !accepted_, "Goal NOT accepted and sending feedback");
  int total_goals = feedback->action_execution_status.size();
  int completed_goals = 0;
  for (const auto &action : feedback->action_execution_status)
  {
    if (action.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING)
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Executing action: %s with progress: %f", action.action_full_name.c_str(), action.completion);
    }
    if (action.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED)
      ++completed_goals;
  }
  float progress = total_goals > 0 ? static_cast<float>(completed_goals) / total_goals : 0.0;
  status_ = "expect_joints";
  // if (progress > 0)
  send_feedback(progress, status_);
}

void ExecuteBottom::resultCallback(const GoalHandleRunBottom::WrappedResult & result)
{
  bool finished = result.code == rclcpp_action::ResultCode::SUCCEEDED, publish_msg_ = false;
  auto message = jigless_planner_interfaces::msg::JointStatus();
  message.joints.reserve(result.result->failed_joints.joints.size());
  message.status.reserve(result.result->failed_joints.joints.size());
  for (std::size_t i = 0; i < result.result->failed_joints.joints.size(); ++i) {
    message.joints.emplace_back(result.result->failed_joints.joints[i]);
    message.status.emplace_back(result.result->failed_joints.status[i]);
    // if(!publish_msg_)
    //   publish_msg_ = result.result->failed_joints.status[i];
  }
  switch (result.code)
  {
  case rclcpp_action::ResultCode::SUCCEEDED:
    status_ = "Execute Bottom completed";
    RCLCPP_INFO(this->get_logger(), status_.c_str());
    break;
  case rclcpp_action::ResultCode::ABORTED:
    status_ = "expect_joints";
    RCLCPP_ERROR(this->get_logger(), status_.c_str());
    break;
  case rclcpp_action::ResultCode::CANCELED:
    status_ = "expect_joints";
    RCLCPP_WARN(this->get_logger(), status_.c_str());
    break;
  }
  // if (publish_msg_) {
    joint_status_publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Publishing failed joint status of size: %zu", message.joints.size());
  // }
  finished_ = true;
  finish(finished, 1.0, status_);
  accepted_ = false;
  status_.clear();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExecuteBottom>();
  node->set_parameter(rclcpp::Parameter("action_name", "execute"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}