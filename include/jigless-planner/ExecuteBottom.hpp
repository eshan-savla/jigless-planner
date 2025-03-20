#ifndef JIGLESS_PLANNER_EXECUTEBOTTOM_HPP
#define JIGLESS_PLANNER_EXECUTEBOTTOM_HPP

#include <chrono>
#include <thread>
#include <memory>
#include <string>
#include <mutex>
#include <map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <plansys2_executor/ActionExecutorClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>

#include "jigless_planner_interfaces/action/run_bottom.hpp"
#include "jigless_planner_interfaces/msg/joint_status.hpp"

class ExecuteBottom : public plansys2::ActionExecutorClient
{
  public:
    using RunBottom = jigless_planner_interfaces::action::RunBottom;
    using GoalHandleRunBottom = rclcpp_action::ClientGoalHandle<RunBottom>;
    
    ExecuteBottom();

  private:
    // bool publish_msg_ = false;
    // std::map<std::string, bool> joint_status_;
    // std::mutex joint_status_mutex_;

    rclcpp::CallbackGroup::SharedPtr action_group_;
    rclcpp::CallbackGroup::SharedPtr publisher_group_;
    rclcpp::PublisherOptions publisher_opts_;

    rclcpp::Publisher<jigless_planner_interfaces::msg::JointStatus>::SharedPtr joint_status_publisher_;
    rclcpp_action::Client<RunBottom>::SharedPtr action_client_;

    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_future<GoalHandleRunBottom::SharedPtr> future_executor_goal_handle_;

    std::vector<std::string> getCommandedJoints();
    void jointStatusPublisher();
    void feedbackCallback(GoalHandleRunBottom::SharedPtr, 
      const std::shared_ptr<const RunBottom::Feedback> & feedback);
    void resultCallback(const GoalHandleRunBottom::WrappedResult & result);
    void do_work() override;
    // void responseCallback(std::shared_future<GoalHandleRunBottom::SharedPtr> future);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & previous_state) override;
};

#endif  // JIGLESS_PLANNER_EXECUTEBOTTOM_HPP