#include "jigless-planner/TopControllerNode.hpp"
#include <plansys2_pddl_parser/Utils.h>
#include "TopControllerNode.hpp"

namespace jigless_planner
{

  TopControllerNode::TopControllerNode(const std::string &node_name) : rclcpp::Node(node_name), state_(STARTING)
  {
    init();
  }

  TopControllerNode::~TopControllerNode()
  {
    RCLCPP_INFO(this->get_logger(), "Destroying top controller node");
  }

  void TopControllerNode::init()
  {
    lifecycle_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    action_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    change_state_client_ = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "bottom_controller_node/change_state", rmw_qos_profile_services_default, lifecycle_group);
    get_state_client_ = this->create_client<lifecycle_msgs::srv::GetState>(
      "bottom_controller_node/get_state", rmw_qos_profile_services_default, lifecycle_group);
    this->declare_parameter("problem_file_path", "/home/mdh-es/multirobot_ws/src/jigless-planner/pddl/top_welding_problem.pddl");
  }

  bool TopControllerNode::init_knowledge()
  {
    using namespace std::literals::chrono_literals;
    std::string problem_file_path = this->get_parameter("problem_file_path").as_string();
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
    auto result = add_problem_client_->async_send_request(request).future.share();
    auto status = result.wait_for(3s);
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

  void TopControllerNode::set_state(unsigned int state) {
    using namespace std::literals::chrono_literals;
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = state;
    if (!change_state_client_->wait_for_service(3s)) {
      RCLCPP_ERROR(this->get_logger(), "Service %s not available", change_state_client_->get_service_name());
      return;
    }
    auto future_result = change_state_client_->async_send_request(request).future.share();
    auto status = future_result.wait_for(3s);
    if (status != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Server timeout while setting state for bottom controller node");
      return;
    }
    if (future_result.get()->success) {
      RCLCPP_INFO(this->get_logger(), "State changed successfully");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to change state for bottom controller node");
    }
  }

  void TopControllerNode::step()
  {
    // Implement the state machine logic here
    switch (state_) {
      case STARTING:{
        if (init_knowledge()) {
          set_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
          state_ = READY;
          RCLCPP_INFO(this->get_logger(), "Top controller node configured and ready")
        }
        break;
      }
      case READY: {
      // Perform actions for the READY state
        if (!goal_joints.empty()){
          set_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
          std::stringstream goal_ss;
          goal_ss << "(:goal (and ";
          for (const auto &joint : goal_joints) {
            goal_ss << "(welded "<< joint << ") ";
          }
          goal_ss << "))";
          RCLCPP_INFO(this->get_logger(), "Received goal request");
          problem_expert_->setGoal(goal_ss.str());
          std::string domain = domain_expert_->getDomain();
          std::string problem = problem_expert_->getProblem();
          auto plan = planner_client->getPlan(domain, problem);
          if (!plan.has_value()) {
            RCLCPP_ERROR_STREAM(this->get_logger(), "Could not find plan to reach goal " << goal_ss << std::endl); 
            RCLCPP_WARN(this->get_logger(), "Clearing and waiting for new goal");
            problem_expert_->clearGoal();
            goal_joints.clear();
            set_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
          }
          if (executor_client_->start_plan_execution(plan.value())) {
            RCLCPP_INFO(this->get_logger(), "Plan started");
            state_ = RUNNING;
          } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to start plan execution");
          }
        }
        break;
      }
      case RUNNING: {
        // Perform actions for the RUNNING state
        
      }
        break;
      case PAUSED:
        // Perform actions for the PAUSED state
        break;
      case STOPPED:
        // Perform actions for the STOPPED state
        break;
    }
  }
}