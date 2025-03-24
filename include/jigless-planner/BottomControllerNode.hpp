#ifndef JIGLESS_PLANNER__BOTTOM_CONTROLLER_NODE_HPP_
#define JIGLESS_PLANNER__BOTTOM_CONTROLLER_NODE_HPP_

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
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <std_msgs/msg/string.hpp>

#include <plansys2_msgs/srv/add_problem.hpp>
#include <plansys2_msgs/msg/action_execution_info.hpp>
#include <plansys2_msgs/msg/plan.hpp>

#include <plansys2_domain_expert/DomainExpertClient.hpp>
#include <plansys2_executor/ExecutorClient.hpp>
#include <plansys2_planner/PlannerClient.hpp>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>
#include <plansys2_pddl_parser/Utils.h>

#include "jigless_planner_interfaces/action/run_bottom.hpp"
#include "jigless_planner_interfaces/msg/joint_status.hpp"



namespace jigless_planner
{
  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class BottomControllerNode : public rclcpp_lifecycle::LifecycleNode
  {
    public:
    using RunBottom = jigless_planner_interfaces::action::RunBottom;
    using GoalHandleRunBottom = rclcpp_action::ServerGoalHandle<RunBottom>;

    explicit BottomControllerNode(const std::string &node_name, bool intra_process_comms = false);

  private:
    bool activated_ = false, started_ = false, cancel_ = false;
    std::mutex activated_mutex_;
    plansys2::Goal interim_goal_;
    std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
    std::shared_ptr<plansys2::PlannerClient> planner_client_;
    std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
    std::shared_ptr<plansys2::ExecutorClient> executor_client_;
    rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedPtr add_problem_client_;

    std::shared_ptr<rclcpp_action::Server<RunBottom>> action_server_;
    rclcpp::CallbackGroup::SharedPtr timer_group_;
    rclcpp::CallbackGroup::SharedPtr service_group_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<GoalHandleRunBottom> current_goal_handle_;

    void init();
    bool init_knowledge();
    bool response_callback(rclcpp::Client<plansys2_msgs::srv::AddProblem>::SharedFuture future);
    jigless_planner_interfaces::msg::JointStatus get_unfinished_action_args(
      const std::vector<plansys2_msgs::msg::ActionExecutionInfo> & result,
      const std::string & action_name = "weld");

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const RunBottom::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandleRunBottom> goal_handle);
    void handle_accepted(
      const std::shared_ptr<GoalHandleRunBottom> goal_handle);
    void execute(const std::shared_ptr<GoalHandleRunBottom> goal_handle);
    void check_action();
    void deactivate_node();

    CallbackReturnT on_configure(
        const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturnT on_activate(
        const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturnT on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturnT on_cleanup(
        const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturnT on_shutdown(
        const rclcpp_lifecycle::State &previous_state) override;
    // CallbackReturnT on_error(
    //     const rclcpp_lifecycle::State &previous_state) override;
  };
}

#endif  // JIGLESS_PLANNER__BOTTOM_CONTROLLER_NODE_HPP_