#include "jigless-planner/BottomControllerNode.hpp"
#include <plansys2_pddl_parser/Utils.h>

namespace jigless_planner
{
  BottomControllerNode::BottomControllerNode(const std::string &node_name, bool intra_process_comms) :
    rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
  {};

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
    this->declare_parameter("bottom_problem_file_path", "/home/mdh-es/multirobot_ws/src/jigless-planner/pddl/weldcell_problem_no_workpiece.pddl");
    add_problem_client_ = this->create_client<plansys2_msgs::srv::AddProblem>("problem_expert/add_problem");
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
    while(!add_problem_client_->wait_for_service(std::chrono::seconds(1))){
      if (!rclcpp::ok()){
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    auto future = add_problem_client_->async_send_request(request).future.share();
    using namespace std::literals::chrono_literals;
    auto status = future.wait_for(1s);
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

  bool BottomControllerNode::response_callback(rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedFuture future)
  {
  };

  CallbackReturnT BottomControllerNode::on_activate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Activating bottom controller node");
    LifecycleNode::on_activate(previous_state);
    activated_ = true;
    return CallbackReturnT::SUCCESS;
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
    if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
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
    return rclcpp_action::CancelResponse::ACCEPT;
  };

  void BottomControllerNode::handle_accepted(
    const std::shared_ptr<GoalHandleRunBottom> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Accepted goal");
    std::thread(
      std::bind(&BottomControllerNode::execute, this, goal_handle)).detach();
  };

  void BottomControllerNode::execute(const std::shared_ptr<GoalHandleRunBottom> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<RunBottom::Feedback>();
    std::stringstream goal_str;
    goal_str << "(:goal (and ";
    switch (goal->operation)
    {
      case RunBottom::Goal::START: {
        RCLCPP_INFO(this->get_logger(), "Setting joints as goal");
        for (const auto & joint : goal->joints.joints) {
          goal_str << "(welded " << joint << ") ";
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
          goal_str << "(welded " << joint << ") ";
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
    }

    auto result = std::make_shared<RunBottom::Result>();
    while (rclcpp::ok() && activated_) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Cancelling goal");
        executor_client_->cancel_plan_execution();
        result->success = false;
        result->failed_joints = get_unfinished_action_args(executor_client_->getFeedBack().action_execution_status);
        goal_handle->canceled(result);
        break;
      }
      feedback->action_execution_status = executor_client_->getFeedBack().action_execution_status;
      goal_handle->publish_feedback(feedback);
      if (executor_client_->execute_and_check_plan())
      { // Plan finished
        auto plan_result = executor_client_->getResult();
        result->success = plan_result.value().success;
        result->failed_joints = get_unfinished_action_args(plan_result.value().action_execution_status);
        if (plan_result.value().success) {
          RCLCPP_INFO(this->get_logger(), "Plan successfully finished");
          goal_handle->succeed(result);
          break;
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Plan finished with error");
          goal_handle->abort(result);
          break;
        }
      }
    }
  };

  CallbackReturnT BottomControllerNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating bottom controller node");
    activated_ = false;
    interim_goal_ = problem_expert_->getGoal();
    // Clear goal
    problem_expert_->clearGoal(); // Should it though?
    LifecycleNode::on_deactivate(previous_state);
    return CallbackReturnT::SUCCESS;
  }

  CallbackReturnT BottomControllerNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
  {
    RCLCPP_INFO(this->get_logger(), "Cleaning up bottom controller node");
    activated_ = false;

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