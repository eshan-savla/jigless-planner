#include "jigless-planner/BottomControllerNode.hpp"
#include <plansys2_pddl_parser/Utils.h>

namespace jigless_planner
{
  BottomControllerNode::BottomControllerNode(const std::string &node_name, bool intra_process_comms) :
    rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {
    timer_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    this->declare_parameter("bottom_problem_file_path", "/home/mdh-es/multirobot_ws/src/jigless-planner/pddl/weldcell_problem_no_workpiece.pddl");
  };

  CallbackReturnT BottomControllerNode::on_configure(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Configuring bottom controller node");
    LifecycleNode::on_configure(previous_state);
    init();
    if (init_knowledge()) {
      RCLCPP_INFO(this->get_logger(), "Knowledge initialized");
      return CallbackReturnT::SUCCESS;
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize knowledge");
      return CallbackReturnT::FAILURE;
    };
  };

  void BottomControllerNode::init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    action_server_ = rclcpp_action::create_server<RunBottom>(
      this,
      "run_bottom",
      std::bind(&BottomControllerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&BottomControllerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&BottomControllerNode::handle_accepted, this, std::placeholders::_1));
    add_problem_client_ = this->create_client<plansys2_msgs::srv::AddProblem>(
      "problem_expert/add_problem", rmw_qos_profile_services_default, service_group_);
    {
      using namespace std::chrono_literals;
      timer_ = this->create_wall_timer(100ms, std::bind(&BottomControllerNode::check_action, this),
        timer_group_);
    }
  };

  bool BottomControllerNode::init_knowledge()
  {
    std::string problem_file_path = this->get_parameter("bottom_problem_file_path").as_string();
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
    while(!add_problem_client_->wait_for_service(std::chrono::seconds(5))){
      if (!rclcpp::ok()){
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto future = add_problem_client_->async_send_request(request).future.share();
    using namespace std::literals::chrono_literals;
    auto status = future.wait_for(5s);
    if (status == std::future_status::ready) {
    auto result = future.get();
    if (result->success){
      RCLCPP_INFO(this->get_logger(), "Problem added");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Error adding problem");
    }
    return result->success;
    }
    return false;
  };

  CallbackReturnT BottomControllerNode::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    std::lock_guard<std::mutex> lock(activated_mutex_);
    RCLCPP_INFO(this->get_logger(), "Activating bottom controller node");
    auto return_status = LifecycleNode::on_activate(previous_state);
    if (return_status == CallbackReturnT::SUCCESS)
      activated_ = true;
    return return_status;
  }

  rclcpp_action::GoalResponse BottomControllerNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RunBottom::Goal> goal)
  {
    (void)uuid;
    std::stringstream goal_ss;
    for (const auto &joint : goal->joints.joints) {
      goal_ss << joint << " ";
    }
    RCLCPP_INFO(this->get_logger(), "Received goal request for joints: %s", goal_ss.str().c_str());
    if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      return rclcpp_action::GoalResponse::REJECT;
    }
  };

  rclcpp_action::CancelResponse BottomControllerNode::handle_cancel(
    const std::shared_ptr<GoalHandleRunBottom> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    (void)goal_handle;
    cancel_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  void BottomControllerNode::handle_accepted(
    const std::shared_ptr<GoalHandleRunBottom> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Accepted goal");
    current_goal_handle_ = goal_handle;
    std::thread(
      std::bind(&BottomControllerNode::execute, this, goal_handle)).detach();
  };

  void BottomControllerNode::execute(const std::shared_ptr<GoalHandleRunBottom> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    if (goal->joints.joints.empty()) {
      RCLCPP_WARN(this->get_logger(), "No joints provided in goal");
      auto result = std::make_shared<RunBottom::Result>();
      result->success = true;
      result->failed_joints = get_unfinished_action_args(executor_client_->getFeedBack().action_execution_status);
      current_goal_handle_->succeed(result);
      return;
    }
    std::stringstream goal_str;
    goal_str << "(:goal (and";
    switch (goal->operation)
    {
      case RunBottom::Goal::START: {
        RCLCPP_INFO(this->get_logger(), "Setting joints as goal");
        for (const auto & joint : goal->joints.joints) {
          goal_str << " (welded " << joint << ")";
        }
        break;
      }
      case RunBottom::Goal::ADD: {
        RCLCPP_INFO(this->get_logger(), "Adding joints to goal");
        goal_str.clear();
        std::string interim_str = parser::pddl::toString(interim_goal_);
        interim_str.pop_back();
        interim_str.pop_back();
        goal_str << interim_str;
        for (const auto & joint : goal->joints.joints) {
          goal_str << " (welded " << joint << ")";
        }
        break;
      }
      case RunBottom::Goal::REMOVE: {
        RCLCPP_INFO(this->get_logger(), "Removing joints from goal");
        goal_str.clear();
        auto interim_copy = interim_goal_;
        std::vector<int> node_ids;
        for (const auto &joint : goal->joints.joints)
        {
          for (auto & node : interim_copy.nodes) {
            if (node.parameters[0].name == joint) {
              node_ids.push_back(node.node_id);
            }
          }
        }
        for (const auto &node_id : node_ids) {
          interim_copy.nodes.erase(std::remove_if(interim_copy.nodes.begin(),
            interim_copy.nodes.end(), [&node_id](const auto &node)
            { return node.node_id == node_id; }), interim_copy.nodes.end());
          interim_copy.nodes[0].children.erase(std::remove_if(
            interim_copy.nodes[0].children.begin(), interim_copy.nodes[0].children.end(),
            [&node_id](const auto &child_id){ return child_id == node_id; }),
            interim_copy.nodes[0].children.end());
        }
        std::string interim_str = parser::pddl::toString(interim_copy);
        interim_str.pop_back();
        interim_str.pop_back();
        goal_str << interim_str;
        break;
      }
      default:
        break;
    }
    goal_str << "))";
    patch_missing_predicates();
    problem_expert_->setGoal(plansys2::Goal(goal_str.str()));

    auto domain = domain_expert_->getDomain();
    auto problem = problem_expert_->getProblem();
    auto plan = planner_client_->getPlan(domain, problem);

    if (!plan.has_value()) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find plan to reach goal " << parser::pddl::toString(problem_expert_->getGoal()) << std::endl);
      return;
    }

    if (executor_client_->start_plan_execution(plan.value())) {
      RCLCPP_INFO(this->get_logger(), "Plan started");
      started_ = true;
    }
  };

  void BottomControllerNode::check_action()
  {
    if (!started_ || !activated_)
      return;

    auto feedback = std::make_shared<RunBottom::Feedback>();
    auto result = std::make_shared<RunBottom::Result>();

    if (current_goal_handle_->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Cancelling goal");
      executor_client_->cancel_plan_execution();
      result->success = false;
      std::this_thread::sleep_for(std::chrono::seconds(1));
      result->failed_joints = get_unfinished_action_args(executor_client_->getFeedBack().action_execution_status);
      resolve_critical_predicates(executor_client_->getFeedBack().action_execution_status);
      current_goal_handle_->canceled(result);
      // Check if critical predicates are set based on action.
      started_ = false;
      cancel_ = false;
      return;
    }
    feedback->action_execution_status = executor_client_->getFeedBack().action_execution_status;
    current_goal_handle_->publish_feedback(feedback);
    if (!executor_client_->execute_and_check_plan()) { // Plan finished
      started_ = false;
      auto plan_result = executor_client_->getResult();
      result->success = plan_result.value().success;
      result->failed_joints = get_unfinished_action_args(plan_result.value().action_execution_status);
      if (plan_result.value().success) {
        RCLCPP_INFO(this->get_logger(), "Plan successfully finished");
        current_goal_handle_->succeed(result);
        return;
      } else {
        RCLCPP_ERROR(this->get_logger(), "Plan finished with error");
        current_goal_handle_->abort(result);
        return;
      }
    }
    if (!activated_) {
      RCLCPP_ERROR(this->get_logger(), "Goal change recieved");
      started_ = false;
    }
  }

  CallbackReturnT BottomControllerNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating bottom controller node");
    std::thread(std::bind(&BottomControllerNode::deactivate_node, this)).detach();
    return LifecycleNode::on_deactivate(previous_state);
    // return CallbackReturnT::SUCCESS;
  }

  void BottomControllerNode::deactivate_node()
  {
    if (started_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for current goal to finish before deactivating");
      while (started_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    std::unique_lock<std::mutex> lock(activated_mutex_);
    activated_ = false;
    lock.unlock();
    interim_goal_ = problem_expert_->getGoal();
    // Clear goal
    problem_expert_->clearGoal(); // Should it though?
    RCLCPP_INFO(this->get_logger(), "Deactivated bottom controller node");
  }

  void jigless_planner::BottomControllerNode::patch_missing_predicates()
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
  
  void BottomControllerNode::resolve_critical_predicates(
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
    // // Temp to check if this solution is worth it:
    // std::vector<plansys2::Predicate> predicates = problem_expert_->getPredicates();
    for (const auto &critical_predicate : critical_predicates) {
      // bool cont = false;
      // for (const auto &predicate : predicates) {
      //   if (predicate.name == critical_predicate) {
      //     RCLCPP_INFO(this->get_logger(), "Predicate %s found.", predicate.name.c_str());
      //     cont = true;
      //     break;
      //   }
      // }
      // if (cont) {
      //   continue;
      // }
      std::string joint;
      auto rit = result.rbegin();
      while (rit != result.rend()) {
        if (rit->action == action_name && rit->status == plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED) {
          joint = rit->arguments[1];
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
        joint = it->arguments[0];
      }
      std::string predicate = "(" + critical_predicate + " " + joint + ")";
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

  std::vector<std::string> jigless_planner::BottomControllerNode::get_critical_predicates(
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

  void jigless_planner::BottomControllerNode::find_grounded_predicates(
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

  CallbackReturnT BottomControllerNode::on_cleanup(const rclcpp_lifecycle::State &previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up bottom controller node");
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::seconds(5);
    while (started_) {
      auto elapsed_time = std::chrono::steady_clock::now() - start_time;
      if (elapsed_time > timeout) {
        RCLCPP_WARN(this->get_logger(), "Timeout while waiting for current goal to finish");
        return CallbackReturnT::FAILURE;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for current goal to finish before cleaning up");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    std::unique_lock<std::mutex> lock(activated_mutex_);
    activated_ = false;
    lock.unlock();

    // Clean up problem knowledge
    RCLCPP_INFO(this->get_logger(), "Cleaning up problem goal and knowledge");
    problem_expert_->clearGoal();
    problem_expert_->clearKnowledge();

    // Reset clients and action server
    RCLCPP_INFO(this->get_logger(), "Resetting clients and action server");
    domain_expert_.reset();
    planner_client_.reset();
    problem_expert_.reset();
    executor_client_.reset();
    action_server_.reset();
    add_problem_client_.reset();
    LifecycleNode::on_cleanup(previous_state);
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT BottomControllerNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down bottom controller node");

    // Clean up problem knowledge
    problem_expert_->clearGoal();
    problem_expert_->clearKnowledge();

    // Reset clients and action server
    domain_expert_.reset();
    planner_client_.reset();
    problem_expert_.reset();
    executor_client_.reset();
    action_server_.reset();
    add_problem_client_.reset();

    // Deactivate the node
    std::lock_guard<std::mutex> lock(activated_mutex_);
    activated_ = false;
    LifecycleNode::on_shutdown(previous_state);
    return CallbackReturnT::SUCCESS;
  }

  jigless_planner_interfaces::msg::JointStatus BottomControllerNode::get_unfinished_action_args(
    const std::vector<plansys2_msgs::msg::ActionExecutionInfo> & result,
    const std::string & action_name)
  {
    jigless_planner_interfaces::msg::JointStatus unfinished_joints;
    for (const auto & action : result) {
      if (action.action != action_name)
        continue;
      unfinished_joints.joints.insert(unfinished_joints.joints.end(), action.arguments.begin(), action.arguments.end());
      unfinished_joints.status.emplace_back(true ? action.status != plansys2_msgs::msg::ActionExecutionInfo::SUCCEEDED : false);
    }
    return unfinished_joints;
  }
}

int main(int argc, char **argv)
{
rclcpp::init(argc, argv);
auto node = std::make_shared<jigless_planner::BottomControllerNode>("bottom_controller_node");
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node->get_node_base_interface());
executor.spin();

rclcpp::shutdown();
return 0;
}