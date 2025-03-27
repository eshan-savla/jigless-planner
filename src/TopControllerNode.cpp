#include <plansys2_pddl_parser/Utils.h>
#include "jigless-planner/TopControllerNode.hpp"

namespace jigless_planner
{

  TopControllerNode::TopControllerNode(const std::string &node_name) : rclcpp::Node(node_name),
    state_(STARTING), step_duration_(std::chrono::milliseconds(100))
  {
    init();
  }

  TopControllerNode::~TopControllerNode()
  {
    RCLCPP_INFO(this->get_logger(), "Destroying top controller node");
    if(step_timer_)
      step_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Stopping plan execution");
    if (executor_client_)
      executor_client_->cancel_plan_execution();
    if (problem_expert_) {
      problem_expert_->clearGoal();
      problem_expert_->clearKnowledge();
    }
    RCLCPP_INFO(this->get_logger(), "Shutting down bottom controller node");
    if (change_state_client_)
      set_state(lifecycle_msgs::msg::Transition::TRANSITION_DESTROY);
  }

  void TopControllerNode::init()
  {
    lifecycle_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    interactor_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    add_problem_client_ = this->create_client<plansys2_msgs::srv::AddProblem>(
      "problem_expert/add_problem");
    this->declare_parameter("bottom_ns", rclcpp::PARAMETER_STRING);
    this->declare_parameter("bottom_controller_name", rclcpp::PARAMETER_STRING);
    std::string bottom_namespace = this->get_parameter("bottom_ns").as_string();
    std::string bottom_controller_node = this->get_parameter("bottom_controller_name").as_string();
    change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "/" + bottom_namespace + "/" + bottom_controller_node + "/change_state",
      rmw_qos_profile_services_default, lifecycle_group);
    get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
      "/" + bottom_namespace + "/" + bottom_controller_node + "/get_state",
      rmw_qos_profile_services_default, lifecycle_group);
    auto qos_setting = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_setting.reliable();
    auto subscriber_opt = rclcpp::SubscriptionOptions();
    subscriber_opt.callback_group = action_group;
    joint_status_subscriber_ = this->create_subscription<jigless_planner_interfaces::msg::JointStatus>(
      "failed_joints", qos_setting, std::bind(
      &TopControllerNode::jointCallback, this, std::placeholders::_1), subscriber_opt);
    interact_top_service_ = this->create_service<jigless_planner_interfaces::srv::InteractTop>("interact_top",
      std::bind(&TopControllerNode::topServiceCallback, this, std::placeholders::_1,
        std::placeholders::_2), rmw_qos_profile_services_default, interactor_group);
    this->declare_parameter("top_problem_file_path", "/home/mdh-es/multirobot_ws/src/jigless-planner/pddl/top_welding_problem.pddl");
    step_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(step_duration_), std::bind(&TopControllerNode::step, this), executor_group);
  }

  bool TopControllerNode::initKnowledge()
  {
    using namespace std::literals::chrono_literals;
    std::string problem_file_path = this->get_parameter("top_problem_file_path").as_string();
    std::ifstream problem_file(problem_file_path);
    if (!problem_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open problem file: %s", problem_file_path.c_str());
      return false;
    }

    std::stringstream buffer;
    buffer << problem_file.rdbuf();
    auto request = std::make_shared<plansys2_msgs::srv::AddProblem::Request>();
    request->problem = buffer.str();
    problem_file.close();
    while (!add_problem_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto future = add_problem_client_->async_send_request(request).future.share();
    auto status = future.wait_for(3s);
    if (status == std::future_status::ready) {
      auto result = future.get();
      if (result->success){
        RCLCPP_INFO(this->get_logger(), "Problem added");
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error adding problem");
      }
      RCLCPP_ERROR(this->get_logger(), "Service timed out waiting for response");
      return result->success;
    } 
    return false;
  };

  void TopControllerNode::resetTimer(std::chrono::milliseconds duration)
  {
    step_duration_ = duration;
    step_timer_->cancel();
    step_timer_ = this->create_wall_timer(
      step_duration_, std::bind(&TopControllerNode::step, this), executor_group);
  }

  void TopControllerNode::jointCallback(const jigless_planner_interfaces::msg::JointStatus &msg)
  {
    std::lock_guard<std::mutex> lock(failed_joints_mutex_);
    RCLCPP_WARN(this->get_logger(), "Some joints failed to weld");
    for (std::size_t i = 0; i < msg.joints.size(); ++i) {
      if (!pause_)
        pause_ = msg.status[i];
      failed_joints[msg.joints[i]] = msg.status[i];
    }
    pause_ = pause_ && !goal_was_changed_ ? pause_ : false;
    goal_was_changed_ = false;
  }

  void TopControllerNode::topServiceCallback(
    const std::shared_ptr<jigless_planner_interfaces::srv::InteractTop::Request> request,
    const std::shared_ptr<jigless_planner_interfaces::srv::InteractTop::Response> response)
  {
    switch (request->operation)
    {
    case jigless_planner_interfaces::srv::InteractTop::Request::START: {
      goal_joints = std::move(request->joints.joints);
      break;
    }
    case jigless_planner_interfaces::srv::InteractTop::Request::ADD: {
      goal_joints.reserve(goal_joints.size() + request->joints.joints.size());
      for (std::size_t i = 0; i < request->joints.joints.size(); ++i) {
        goal_joints.emplace_back(request->joints.joints[i]);
      }
      goal_changed_ = true;
      break;
    }
    case jigless_planner_interfaces::srv::InteractTop::Request::REMOVE: {
      for (std::size_t i = 0; i < request->joints.joints.size(); ++i) {
        problem_expert_->removePredicate(plansys2::Predicate("(commanded " + request->joints.joints[i] + ")"));
        goal_joints.erase(std::remove(goal_joints.begin(), goal_joints.end(), request->joints.joints[i]), goal_joints.end());
      }
      goal_changed_ = true;
      break;
    }
    case jigless_planner_interfaces::srv::InteractTop::Request::STOP: {
      std::lock_guard<std::mutex> lock(interaction_mutex_);
      cancel_ = true;
    }
    default:
      break;
    }
    response->success = true;
  }

  unsigned int TopControllerNode::get_state()
  {
    using namespace std::literals::chrono_literals;
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    if (!get_state_client_->wait_for_service(3s)) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available", get_state_client_->get_service_name());
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
    auto future_result = get_state_client_->async_send_request(request).future.share();
    auto status = future_result.wait_for(3s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Server timeout while getting current state for bottom controller node");
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (future_result.get()) {
      return future_result.get()->current_state.id;
    }
    RCLCPP_ERROR(this->get_logger(), "Failed to get current state for bottom controller node");
    return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
  }

  bool TopControllerNode::set_state(unsigned int state) {
    using namespace std::literals::chrono_literals;
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = state;
    if (!change_state_client_->wait_for_service(3s)) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available", change_state_client_->get_service_name());
      return false;
    }
    auto future_result = change_state_client_->async_send_request(request).future.share();
    auto status = future_result.wait_for(3s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Server timeout while setting state for bottom controller node");
      return false;
    }
    if (future_result.get()->success) {
      RCLCPP_INFO(this->get_logger(), "State changed successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to change state for bottom controller node");
    }
    return future_result.get()->success;
  }

  std::unique_ptr<std::vector<plansys2_msgs::msg::ActionExecutionInfo>> TopControllerNode::get_executing_actions(
    const std::vector<plansys2_msgs::msg::ActionExecutionInfo> &feedback)
  {
    auto executing_actions = std::make_unique<std::vector<plansys2_msgs::msg::ActionExecutionInfo>>();
    for (const auto &action : feedback) {
      if (action.status == plansys2_msgs::msg::ActionExecutionInfo::EXECUTING) {
        executing_actions->push_back(action);
      }
    }
    return executing_actions;
  }

  void TopControllerNode::step()
  {
    // Implement the state machine logic here
    switch (state_) {
      case STARTING: {
        if (initKnowledge()) {
          if (!set_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE)){
            resetTimer(std::chrono::milliseconds(5000));
            break;
          }
          if (step_duration_ > std::chrono::milliseconds(100)) {
            resetTimer(std::chrono::milliseconds(100));
          }
          state_ = READY;
          RCLCPP_INFO(this->get_logger(), "Top controller node configured and ready");
        }
        break;
      }
      case READY: {
      // Perform actions for the READY state
        if (cancel_)
          cancel_ = false;
        if (!goal_joints.empty()){
          if (!set_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
            resetTimer(std::chrono::milliseconds(1000));
            break;
          }
          if (step_duration_ > std::chrono::milliseconds(100)) {
            resetTimer(std::chrono::milliseconds(100));
          }
          std::stringstream goal_ss;
          goal_ss << "(:goal (and";
          for (const auto &joint : goal_joints) {
            goal_ss << " (welded "<< joint << ")";
          }
          goal_ss << "))";
          std::string s = goal_ss.str();
          RCLCPP_INFO(this->get_logger(), "Received goal request");
          problem_expert_->setGoal(plansys2::Goal(goal_ss.str()));
          std::string domain = domain_expert_->getDomain();
          std::string problem = problem_expert_->getProblem();
          auto plan = planner_client_->getPlan(domain, problem);
          if (!plan.has_value()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find plan to reach goal " << goal_ss.str() << std::endl); 
            RCLCPP_WARN(this->get_logger(), "Clearing and waiting for new goal");
            problem_expert_->clearGoal();
            goal_joints.clear();
            set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
          }
          if (executor_client_->start_plan_execution(plan.value())) {
            RCLCPP_INFO(this->get_logger(), "Plan started");
            pause_ = false;
            cancel_ = false;
            goal_changed_ = false;
            state_ = RUNNING;
            RCLCPP_INFO(this->get_logger(), "Switching to RUNNING state");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to start plan execution");
            RCLCPP_WARN(this->get_logger(), "Clearing and waiting for new goal");
            problem_expert_->clearGoal();
            goal_joints.clear();
            set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
          }
        }
        break;
      }
      case RUNNING: {
        // Perform actions for the RUNNING state
        if (!executor_client_->execute_and_check_plan()) {
          auto plan_result = executor_client_->getResult();
          if (!plan_result.has_value()) {
            RCLCPP_ERROR(this->get_logger(), "Plan finished with error");
            RCLCPP_WARN(this->get_logger(), "Clearing and waiting for new goal");
            problem_expert_->clearGoal();
            goal_joints.clear();
            set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
            state_ = READY;
            break;
          }
          if (plan_result.value().success) {
            RCLCPP_INFO(this->get_logger(), "Plan successfully finished");
          } else {
            RCLCPP_ERROR(this->get_logger(), "Plan finished with error");
          }
          RCLCPP_INFO(this->get_logger(), "Clearing goal");
          problem_expert_->clearGoal();
          goal_joints.clear();
          RCLCPP_INFO(this->get_logger(), "Deactivating bottom controller");
          set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
          RCLCPP_INFO(this->get_logger(), "Switching to READY state");
          state_ = READY;
          break;
        }
        if (goal_changed_) {
          RCLCPP_INFO(this->get_logger(), "Goal changed, stopping execution");
          executor_client_->cancel_plan_execution();
          state_ = STOPPED;
          break;
        }
        {
          std::lock_guard<std::mutex> lock(failed_joints_mutex_);
          if (pause_) {
            state_ = PAUSED;
            break;
          }
        }
        if (cancel_) {
          RCLCPP_INFO(this->get_logger(), "Stopping execution");
          state_ = CANCELLED;
          break;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Running plan execution");
        auto current_actions = get_executing_actions(executor_client_->getFeedBack().action_execution_status);
        for (const auto &action : *current_actions) {
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
          "Executing action: %s with progress: %f", action.action_full_name.c_str(),
          action.completion);
        }
        break;
      }
      case PAUSED: {
        // Perform actions for the PAUSED state
        RCLCPP_WARN(this->get_logger(), "Paused as some joints failed to weld");
        RCLCPP_WARN(this->get_logger(), "Cancelling plan execution");
        executor_client_->cancel_plan_execution();
        std::unique_lock<std::mutex> joints_lock(failed_joints_mutex_);
        std::map<std::string, bool> failed_joints_cp = std::move(failed_joints); // Check if failed_joints is empty after this
        pause_ = false;
        joints_lock.unlock();
        for (const auto &pair : failed_joints_cp) {
          if (pair.second) {
            RCLCPP_WARN(this->get_logger(), "Failed joint: %s", pair.first.c_str());
            RCLCPP_WARN(this->get_logger(), "Resetting joint status");
            problem_expert_->removePredicate(plansys2::Predicate("(welded " + pair.first + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(not_welded " + pair.first + ")"));
            problem_expert_->removePredicate(plansys2::Predicate("(commanded " + pair.first + ")"));
          } else {
            RCLCPP_INFO(this->get_logger(), "Joint %s is ok", pair.first.c_str());
            goal_joints.erase(std::remove(goal_joints.begin(), goal_joints.end(), pair.first), goal_joints.end());
            problem_expert_->removePredicate(plansys2::Predicate("(not_welded " + pair.first + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(welded " + pair.first + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(commanded " + pair.first + ")"));
          }
        }
        RCLCPP_INFO(this->get_logger(), "Updated predicates");
        RCLCPP_INFO(this->get_logger(), "Clearing goal for replanning under new circumstances");
        problem_expert_->clearGoal();
        RCLCPP_INFO(this->get_logger(), "Resetting execution status");
        problem_expert_->addPredicate(plansys2::Predicate("(not_executing)"));
        state_ = READY;
        RCLCPP_INFO(this->get_logger(), "Switching to READY state");
        break;
      }
      case STOPPED: {
        // Perform actions for the STOPPED state
        RCLCPP_INFO(this->get_logger(), "Deactivating bottom controller");
        set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        RCLCPP_INFO(this->get_logger(), "Clearing goal");
        problem_expert_->clearGoal();
        RCLCPP_INFO(this->get_logger(), "Resetting execution status");
        auto predicates = problem_expert_->getPredicates();
        auto dom_preds = domain_expert_->getPredicates();
        bool success = problem_expert_->addPredicate(plansys2::Predicate("(not_executing)"));
        RCLCPP_INFO(this->get_logger(), "Switching to READY state");
        state_ = READY;
        goal_changed_ = false;
        goal_was_changed_ = true;
        break;
      }
      case CANCELLED: {
        // Perform actions for the CANCELLED state
        RCLCPP_INFO(this->get_logger(), "Cancelling plan execution");
        executor_client_->cancel_plan_execution();
        RCLCPP_INFO(this->get_logger(), "Deactivating bottom controller");
        set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        RCLCPP_INFO(this->get_logger(), "Clearing goal");
        problem_expert_->clearGoal();
        goal_joints.clear();
        RCLCPP_INFO(this->get_logger(), "Resetting execution status");
        problem_expert_->addPredicate(plansys2::Predicate("(not_executing)"));
        RCLCPP_INFO(this->get_logger(), "Switching to READY state");
        state_ = READY;
        std::unique_lock<std::mutex> lock(interaction_mutex_);
        lock.unlock();
        break;
      }
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<jigless_planner::TopControllerNode>("top_controller_node");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}