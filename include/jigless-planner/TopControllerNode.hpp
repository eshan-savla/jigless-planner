#ifndef JIGLESS_PLANNER_TOPCONTROLLERNODE_HPP
#define JIGLESS_PLANNER_TOPCONTROLLERNODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <utility>
#include <fstream>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/srv/get_state.hpp>

#include <plansys2_domain_expert/DomainExpertClient.hpp>
#include <plansys2_executor/ExecutorClient.hpp>
#include <plansys2_planner/PlannerClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>

#include "jigless_planner_interfaces/msg/joint_status.hpp"
#include "jigless_planner_interfaces/srv/interact_top.hpp"

namespace jigless_planner
{
  class TopControllerNode : public rclcpp::Node
  {
  public:
    explicit TopControllerNode(const std::string &node_name);
    ~TopControllerNode();

  private:
    bool goal_changed_ = false, pause_ = false;
    std::vector<std::string> goal_joints;
    std::map<std::string, bool> failed_joints;
    std::mutex failed_joints_mutex_;
    typedef enum
    {
      STARTING,
      READY,
      RUNNING,
      PAUSED,
      STOPPED
    } StateType;
    StateType state_;
    
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    
    // Add Server for goal adding/appending/removal etc

    std::shared_ptr<rclcpp::CallbackGroup> lifecycle_group;
    std::shared_ptr<rclcpp::CallbackGroup> action_group;
    std::shared_ptr<rclcpp::CallbackGroup> interactor_group;
    std::shared_ptr<rclcpp::CallbackGroup> executor_group;

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
    rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedPtr add_problem_client_;
    rclcpp::Subscription<jigless_planner_interfaces::msg::JointStatus>::SharedPtr joint_status_subscriber_;
    rclcpp::Service<jigless_planner_interfaces::srv::InteractTop>::SharedPtr interact_top_service_;
    rclcpp::TimerBase::SharedPtr step_timer_;
    std::chrono::milliseconds step_duration_;

    void init();
    bool initKnowledge();
    void resetTimer(std::chrono::milliseconds duration);
    std::unique_ptr<std::vector<plansys2_msgs::msg::ActionExecutionInfo>> get_executing_actions(
      const std::vector<plansys2_msgs::msg::ActionExecutionInfo> &feedback);
    void jointCallback(const jigless_planner_interfaces::msg::JointStatus & msg);
    void topServiceCallback(
      const std::shared_ptr<jigless_planner_interfaces::srv::InteractTop::Request> request,
      const std::shared_ptr<jigless_planner_interfaces::srv::InteractTop::Response> response);
    void step(); // Runs the whole loop with state change based actions
    // bool response_callback(rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedFuture future);
    // void get_state_callback(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future);
    // void set_state_callback(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future);

    unsigned int get_state();
    bool set_state(unsigned int state);
  };
}

#endif // JIGLESS_PLANNER_TOPCONTROLLERNODE_HPP