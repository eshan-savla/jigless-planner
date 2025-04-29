#include <plansys2_pddl_parser/Utils.h>
#include "jigless-planner/TopControllerNode.hpp"

namespace jigless_planner
{

  TopControllerNode::TopControllerNode(const std::string &node_name) : rclcpp::Node(node_name),
    state_(STARTING), step_duration_(std::chrono::milliseconds(100))
  {
    planning_thread_ = std::thread([this]() {
      while (rclcpp::ok()) {
        std::unique_lock<std::mutex> lock(planning_mutex_);
        planning_cv_.wait(lock, [this]() {return planning_requested_;});
        planning_requested_ = false;
        lock.unlock();
        plan();
      }
    });
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
    std::unique_lock<std::mutex> lock(planning_mutex_);
    planning_requested_ = false;
    planning_cv_.notify_one();
    if (planning_thread_.joinable()) {
      planning_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Top controller node destroyed");
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
    execute_started_subscriber_ = this->create_subscription<std_msgs::msg::Empty>(
      "execute_bottom_started", qos_setting, std::bind(
      &TopControllerNode::executeStartedCallback, this, std::placeholders::_1), subscriber_opt);
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
    for (std::size_t i = 0; i < msg.joints.size(); ++i) {
      if (!pause_)
        pause_ = msg.status[i];
      else
        RCLCPP_WARN(this->get_logger(), "Some joints failed to weld");
      failed_joints[msg.joints[i]] = msg.status[i];
    }
    pause_ = pause_ && !goal_was_changed_ ? pause_ : false;
    goal_was_changed_ = false;
    replan_joints_recieved_ = true;
  }

  void TopControllerNode::executeStartedCallback(const std_msgs::msg::Empty &msg)
  {
    RCLCPP_INFO(this->get_logger(), "Execute bottom started");
    expect_joints_ = true;
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
        feedback_.clear();
        if (cancel_)
          cancel_ = false;
        if (!goal_joints.empty()){
          if (get_state() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
            if (!set_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE)){
              resetTimer(std::chrono::milliseconds(1000));
              break;
            }
          if (get_state() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
            resetTimer(std::chrono::milliseconds(1000));
            break;
          }
          if (step_duration_ > std::chrono::milliseconds(100)) {
            resetTimer(std::chrono::milliseconds(100));
          }
          state_ = PLANNING;
          RCLCPP_INFO(this->get_logger(), "Switching to PLANNING state");
        }
        break;
      }
      case PLANNING: {
        // Perform actions for the PLANNING state
        {
          std::lock_guard<std::mutex> lock(planning_mutex_);
          if (planning_joints != goal_joints) {
            planning_requested_ = true;
          }
          if (planning_requested_) {
            planning_joints = goal_joints;
            planning_cv_.notify_one();
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
        // Save feedback at every check to be used later.
        feedback_ = executor_client_->getFeedBack().action_execution_status;
        if (goal_changed_) {
          RCLCPP_INFO(this->get_logger(), "Goal changed, stopping execution");
          state_ = WAITING;
          break;
        }
        {
          std::lock_guard<std::mutex> lock(failed_joints_mutex_);
          if (pause_ && replan_joints_recieved_) {
            state_ = UPDATING;
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
      case WAITING: {
        RCLCPP_WARN(this->get_logger(), "Cancelling plan execution");
        executor_client_->cancel_plan_execution();
        RCLCPP_INFO(this->get_logger(), "Waiting for replan joints from bottom controller");
        auto start_time = std::chrono::steady_clock::now();
        auto timeout = std::chrono::minutes(5);
        StateType switch_to;
        while (rclcpp::ok())
        {
          if (!expect_joints_) {
            RCLCPP_INFO(this->get_logger(), "No joints expected");            
            switch_to = UPDATING;
            break;
          }
          auto elapsed = std::chrono::steady_clock::now() - start_time;
          if (elapsed > timeout) {
            RCLCPP_WARN(this->get_logger(), "Timeout waiting for failed joints");
            switch_to = CANCELLED;
            break;
          }
          if(replan_joints_recieved_){
            switch_to = UPDATING;
            expect_joints_ = false;
            break;
          }
          if (cancel_) {
            RCLCPP_INFO(this->get_logger(), "Waiting cancelled");
            switch_to = CANCELLED;
            expect_joints_ = false;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        RCLCPP_INFO(this->get_logger(), "Switching to %s state", switch_to == UPDATING ? "UPDATING" : "CANCELLED");
        state_ = switch_to;
        break;
      }
      case UPDATING: {
        // Perform actions for the PAUSED state
        RCLCPP_WARN_EXPRESSION(this->get_logger(), goal_changed_, "Paused as goal changed");
        RCLCPP_WARN_EXPRESSION(this->get_logger(), !goal_changed_, "Paused as some joints failed to weld");
        if (pause_){
          RCLCPP_WARN(this->get_logger(), "Cancelling plan execution");
          executor_client_->cancel_plan_execution();
        }
        if (goal_changed_) {
          RCLCPP_INFO(this->get_logger(), "Deactivating bottom controller");
          set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        }
        std::unique_lock<std::mutex> joints_lock(failed_joints_mutex_);
        std::map<std::string, bool> failed_joints_cp = std::move(failed_joints); // Check if failed_joints is empty after this
        joints_lock.unlock();
        resolve_critical_predicates(feedback_);
        std::string current_pos = getCurrentPosFromAction("execute");
        RCLCPP_INFO(this->get_logger(), "Current position: %s", current_pos.c_str());
        for (const auto &pair : failed_joints_cp) {
          if (pair.second) {
            RCLCPP_WARN(this->get_logger(), "Failed joint: %s", pair.first.c_str());
            RCLCPP_WARN(this->get_logger(), "Resetting joint status");
            problem_expert_->removePredicate(plansys2::Predicate("(welded " + pair.first + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(not_welded " + pair.first + ")"));
            problem_expert_->removePredicate(plansys2::Predicate("(commanded " + pair.first + ")"));
            // Add case where goal_changed_ is checked. If no, change reachable pos. Warn if no new pos and retry from same.
            // Get reachable poses of joint via built-in method which searches through predicates.
            // Determine execute-bottom pos and remove that from joint, if alternatives are there.
            if (!goal_changed_) {
              problem_expert_->removePredicate(plansys2::Predicate("(commandable " + pair.first + ")"));
              RCLCPP_INFO(this->get_logger(), "Changing reachable pos of %s", pair.first.c_str());
              std::vector<std::string> reachable_pos = getReachablePos(pair.first);
              if (reachable_pos.size() > 1) {
                problem_expert_->removePredicate(plansys2::Predicate("(reachable_at " + pair.first +
                  " " + current_pos + ")"));
              }
              else {
                RCLCPP_WARN(this->get_logger(), "No additional reachable positions for joint %s",
                  pair.first.c_str());
                RCLCPP_WARN(this->get_logger(), "Retrying from same position");
              }
            }
          } else {
            RCLCPP_INFO(this->get_logger(), "Joint %s is ok", pair.first.c_str());
            goal_joints.erase(std::remove(goal_joints.begin(), goal_joints.end(), pair.first),
              goal_joints.end());
            problem_expert_->removePredicate(plansys2::Predicate("(not_welded " + pair.first + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(welded " + pair.first + ")"));
            problem_expert_->addPredicate(plansys2::Predicate("(commanded " + pair.first + ")"));
          }
        }
        goal_was_changed_ = goal_changed_ ? true : false;
        goal_changed_ = false;
        pause_ = false;
        replan_joints_recieved_ = false;
        RCLCPP_INFO(this->get_logger(), "Updated predicates");
        RCLCPP_INFO(this->get_logger(), "Clearing goal for replanning under new circumstances");
        problem_expert_->clearGoal();
        RCLCPP_INFO(this->get_logger(), "Resetting execution status");
        problem_expert_->addPredicate(plansys2::Predicate("(not_executing)"));
        planning_joints.clear();
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
        bool success = problem_expert_->addPredicate(plansys2::Predicate("(not_executing)"));
        RCLCPP_INFO(this->get_logger(), "Switching to READY state");
        state_ = READY;
        goal_changed_ = false;
        goal_was_changed_ = true;
        break;
      }
      case CANCELLED: {
        // Perform actions for the CANCELLED state
        if (!cancel_) {
          RCLCPP_DEBUG(this->get_logger(), "Already cancelling");
          break;
        }
        RCLCPP_INFO(this->get_logger(), "Cancelling plan execution");
        executor_client_->cancel_plan_execution();
        RCLCPP_INFO(this->get_logger(), "Resetting execution status");
        problem_expert_->addPredicate(plansys2::Predicate("(not_executing)"));
        RCLCPP_INFO(this->get_logger(), "Deactivating bottom controller");
        set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
        RCLCPP_INFO(this->get_logger(), "Clearing goal and knowledge");
        problem_expert_->clearGoal();
        problem_expert_->clearKnowledge();
        goal_joints.clear();
        planning_joints.clear();
        RCLCPP_INFO(this->get_logger(), "Cleaning up bottom controller");
        std::thread([this]()
        {
          std::unique_lock<std::mutex> lock(interaction_mutex_);
          cancel_ = false;
          lock.unlock();
          while (rclcpp::ok() && get_state() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
            set_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
          }
          RCLCPP_INFO(this->get_logger(), "Bottom controller cleaned up");
          RCLCPP_INFO(this->get_logger(), "Switching to STARTING state");
          state_ = STARTING;
        }).detach();
        break;
      }
    }
  }

  std::string TopControllerNode::getCurrentPosFromAction(const std::string & action_name) const {
    auto last_succeeded_it = std::find_if(feedback_.rbegin(), feedback_.rend(),
                        [action_name](const plansys2_msgs::msg::ActionExecutionInfo &action)
                        {
                          return action.action == action_name &&
                                action.status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED;
                        });
    if (last_succeeded_it == feedback_.rend()) {
      auto first_it = std::find_if(feedback_.begin(), feedback_.end(),
        [action_name](const plansys2_msgs::msg::ActionExecutionInfo &action) {
          return action.action == action_name;
        }
      );
      return first_it != feedback_.end() ? first_it->arguments[0] : "";
    }
    auto forward_it = last_succeeded_it.base();
    auto next_execute_it = std::find_if(forward_it, feedback_.end(),
                      [action_name](const plansys2_msgs::msg::ActionExecutionInfo &action)
                      {
                        return action.action == action_name;
                      });
    return next_execute_it != feedback_.end() ? next_execute_it->arguments[0]
     : last_succeeded_it->arguments[0];
  }

  std::vector<std::string> TopControllerNode::getReachablePos(const std::string &joint_name) {
    std::vector<plansys2::Predicate> predicates = getInstancePredicates("reachable_at", joint_name);
    std::vector<std::string> reachable_positions;
    for (const auto &predicate : predicates) {
      if (predicate.parameters.size() == 2) {
        reachable_positions.push_back(predicate.parameters[1].name);
      }
    }
    auto it = reachable_pos_map_.find(joint_name);
    if (it != reachable_pos_map_.end() && it->second.size() <= reachable_positions.size()) {
      return it->second;
    }
    reachable_pos_map_[joint_name] = reachable_positions;
    return reachable_positions;
  }

  std::vector<plansys2::Predicate> TopControllerNode::getInstancePredicates(
    const std::string &predicate_name, const std::string &instance_name) const {
    std::vector<plansys2::Predicate> predicates = problem_expert_->getPredicates();
    auto it = predicates.begin();
    while (it != predicates.end()) {
      if (it->name == predicate_name &&
          it->parameters[0].name == instance_name) {
        ++it;
      } else {
        it = predicates.erase(it);
      }
    }
    return predicates;
  }

  void jigless_planner::TopControllerNode::patch_missing_predicates()
  {
    // Necessary as some BT nodes are initialised and cancelled immediately after critical predicates are resolved.
    if (expected_crit_preds_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No expected critical predicates found");
      return;
    }
    std::vector<plansys2::Predicate> predicates = problem_expert_->getPredicates();
    for (const auto &predicate : expected_crit_preds_) {
      bool cont = false;
      for (const auto &pred : predicates) {
        // std::string predicate_name = predicate.substr(1, predicate.find(" ") - 1);
        if (predicate.substr(1, predicate.find(" ", 2) - 1) == pred.name) {
          RCLCPP_INFO(this->get_logger(), "Predicate %s found.", pred.name.c_str());
          cont = true;
          break;
        }
      }
      if (cont) {
        continue;
      }
      bool exist_predicate = problem_expert_->existPredicate(plansys2::Predicate(predicate));
      if (!exist_predicate) {
        RCLCPP_INFO(this->get_logger(), "Setting critical predicate %s", predicate.c_str());
        problem_expert_->addPredicate(plansys2::Predicate(predicate));
      } else {
        RCLCPP_INFO(this->get_logger(), "Critical predicate %s already exists", predicate.c_str());
      }
    }
    expected_crit_preds_.clear();
  }

  void TopControllerNode::resolve_critical_predicates(
    const std::vector<plansys2_msgs::msg::ActionExecutionInfo> &result,
    const std::string &action_name)
  {
    // Idea is to check if an instance of critical predicates exist.
    // That means, the predicates set in the effects of the action need to be checked
    // If no instance of it exists, then valid one should be created.

    // How to do:
    // Critical predicates: removed at start and set at end or vice versa
    // Determine the action for which critical predicates exist (Ideally, this should be done once in the beginning and stored)
    // ALTERNATIVE FOR TEMP: Action is known, just determine which is the critical predicate for that action.
    // Determine first successful instance of action and determine the status of the critical predicate
    // Store the effect/corresponding predicate.
    // If this predicate is set/exists, finish and exit method.
    // Else, set this predicate and finish.

    // Method division:
    // 1. Method to determine actions which have critical predicates (optional)
    // 2. Method to determine critical predicates of an action. (Should be utilised by method above outside this function)

    //%TODO: Identify all actions with critical predicates and store them during configuration instead of here.
    std::vector<std::string> critical_predicates = get_critical_predicates(action_name);
    if (critical_predicates.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No critical predicates found");
      return;
    }
    assert(expected_crit_preds_.empty());
    // //Temp to check if this solution is worth it:
    // std::vector<plansys2::Predicate> predicates = problem_expert_->getPredicates();
    for (const auto &critical_predicate : critical_predicates) {
      // bool cont = false;
      // for (const auto &predicate : predicates)
      // {
      //   if (predicate.name == critical_predicate) {
      //     RCLCPP_INFO(this->get_logger(), "Predicate %s found.", predicate.name.c_str());
      //     cont = true;
      //     break;
      //   }
      // }
      // if (cont) {
      //   continue;
      // }
      std::string param;
      auto rit = result.rbegin();
      while (rit != result.rend()) {
        if (rit->action == action_name && rit->status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
          param = rit->arguments[1];
          break;
        }
        ++rit;
      }
      if (rit == result.rend()) {
        RCLCPP_ERROR(this->get_logger(), "No successful %s action found", action_name.c_str());
        RCLCPP_WARN(this->get_logger(), "Using first %s action", action_name.c_str());
        auto it = std::find_if(result.begin(), result.end(),
          [action_name](const plansys2_msgs::msg::ActionExecutionInfo &action)
          { return action.action == action_name; });
        if (it == result.end()) {
          RCLCPP_ERROR(this->get_logger(), "No %s action found", action_name.c_str());
          return;
        }
        param = it->arguments[0];
      }
      std::string predicate = "(" + critical_predicate + " " + param + ")";
      expected_crit_preds_.push_back(predicate);
      // bool exist_predicate = problem_expert_->existPredicate(plansys2::Predicate(predicate));
      // if (!exist_predicate) {
      //   RCLCPP_INFO(this->get_logger(), "Setting critical predicate %s", predicate.c_str());
      //   problem_expert_->addPredicate(plansys2::Predicate(predicate));
      // } else {
      //   RCLCPP_INFO(this->get_logger(), "Critical predicate %s already exists", predicate.c_str());
      // }
    }
  }

  std::vector<std::string> jigless_planner::TopControllerNode::get_critical_predicates(
    const std::string &action_name)
  {
    // Get effects of the action
    // Memoize the action name and the critical predicates
    // Determine if action has at start and at end effects
    // If no, return empty vector
    // If yes, check for similar, negated pair of each predicate in at start
      // How:
      // 1. Get all at start effects of action
      // Ignore/Skip "and"
      // 2. Get ground predicate ideally through recursive function.
      // Function should also return if predicate is negated or not
      // 3. Check if predicate exists in the action at end effects
      // 4. Check if it is the inverse of at start effect
      // 5. If yes, add to vector
    
    if (critical_action_preds_.find(action_name) != critical_action_preds_.end()) {
      return critical_action_preds_[action_name];
    }
    auto action = domain_expert_->getDurativeAction(action_name);
    if (action == nullptr) {
      RCLCPP_ERROR(this->get_logger(), "Action %s not found", action_name.c_str());
      return std::vector<std::string>();
    }
    std::unordered_map<std::string, bool> grounded_at_start;
    std::unordered_set<int> visited_ids_start;
    find_grounded_predicates(action->at_start_effects.nodes, 0, grounded_at_start, visited_ids_start, false);
    std::unordered_map<std::string, bool> grounded_at_end;
    std::unordered_set<int> visited_ids_end;
    find_grounded_predicates(action->at_end_effects.nodes, 0, grounded_at_end, visited_ids_end, false);
    std::vector<std::string> critical_predicates;
    for (const auto &predicate : grounded_at_start) {
      auto it = grounded_at_end.find(predicate.first);
      if (it != grounded_at_end.end())
      {
        if (predicate.second != it->second) {
          critical_predicates.push_back(predicate.first);
        }
      }
    }
    critical_action_preds_[action_name] = critical_predicates;
    return critical_predicates;
  }

  void jigless_planner::TopControllerNode::find_grounded_predicates(
    const std::vector<plansys2_msgs::msg::Node> &effects, const int &id,
    std::unordered_map<std::string, bool> &grounded_predicates,
    std::unordered_set<int> &visited_ids, const bool &negate)
  {
    // POA:
    // 1. Check if effects is empty, if yes, return
    // 2. Loop through effects:
      // 1. Check if effect is "and", call function recursively on children
      // 2. Check if effect is "not", call function recursively on children and set negate to true
      // 3. Check if effect is grounded, if yes, add to grounded_predicates with negate value.

    if (effects.empty()) {
      return;
    }
    for (int i = id; i < effects.size(); ++i)
    {
      if (visited_ids.find(effects.at(i).node_id) != visited_ids.end()) {
        continue;
      }
      visited_ids.insert(effects.at(i).node_id);
      if (effects.at(i).node_type == plansys2_msgs::msg::Node::AND) {
        for (const auto &child : effects.at(i).children) {
          find_grounded_predicates(effects, child, grounded_predicates, visited_ids, negate);
        }
      }
      if (effects.at(i).node_type == plansys2_msgs::msg::Node::NOT) {
        for (const auto &child : effects.at(i).children) {
          find_grounded_predicates(effects, child, grounded_predicates, visited_ids, true);
        }
      }
      if (effects.at(i).node_type == plansys2_msgs::msg::Node::PREDICATE) {
        grounded_predicates[effects.at(i).name] = negate;
      }
    }
  }

  void jigless_planner::TopControllerNode::plan() 
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    patch_missing_predicates();
    std::string goal_string = create_goal_string(planning_joints);
    problem_expert_->setGoal(plansys2::Goal(goal_string));
    std::string domain = domain_expert_->getDomain();
    std::string problem = problem_expert_->getProblem();
    auto future_plan = std::async(std::launch::async, [this, domain, problem]() {
      return planner_client_->getPlan(domain, problem);
    });
    while(future_plan.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
      if (planning_requested_) {
        RCLCPP_INFO(this->get_logger(), "Planning interrupted, restarting");
        return;
      }
      if (cancel_) {
        RCLCPP_INFO(this->get_logger(), "Planning cancelled");
        state_ = CANCELLED;
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    auto plan = future_plan.get();
    if (!plan.has_value()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find plan to reach goal " << goal_string << std::endl); 
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
      state_ = CANCELLED;
    }
  }

  std::string jigless_planner::TopControllerNode::create_goal_string(
    const std::vector<std::string> &goal_joints)
  {
    std::stringstream goal_ss;
    goal_ss << "(:goal (and";
    for (const auto &joint : goal_joints) {
      goal_ss << " (welded "<< joint << ")";
    }
    goal_ss << "))";
    return goal_ss.str();
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