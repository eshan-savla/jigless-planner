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

namespace jigless_planner
{
  class TopControllerNode : public rclcpp::Node
  {
  public:
    explicit TopControllerNode(const std::string &node_name);
    ~TopControllerNode();

  private:
    std::vector<std::string> goal_joints;
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

    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state_client_;
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state_client_;
    rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedPtr add_problem_client_;
    // Add callback for goal server

    void init();
    bool init_knowledge();
    void step(); // Runs the whole loop with state change based actions
    // bool response_callback(rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedFuture future);
    // void get_state_callback(rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future);
    // void set_state_callback(rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future);
    

    unsigned int get_state();
    void set_state(unsigned int state);
  };
}

#endif // JIGLESS_PLANNER_TOPCONTROLLERNODE_HPP