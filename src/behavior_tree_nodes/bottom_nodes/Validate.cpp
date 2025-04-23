#include <string>
#include <iostream>

#include "jigless-planner/behavior_tree_nodes/bottom_nodes/Validate.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace jigless_planner::bottom_actions
{
  namespace weld
  {
    Validate::Validate(
      const std::string & xml_tag_name,
      const std::string & action_name,
      const BT::NodeConfiguration & conf)
    : plansys2::BtActionNode<weld_interfaces::action::Validate>(xml_tag_name, action_name, conf)
    {
      rclcpp::Node::SharedPtr node;
      config().blackboard->get("node", node);
    }

    BT::NodeStatus Validate::on_tick()
    {
      std::string joint;
      getInput("joint", joint);
      goal_.joint = joint;
      RCLCPP_INFO(node_->get_logger(), "Validating joint: %s", joint.c_str());
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus Validate::on_success()
    {
      RCLCPP_INFO(node_->get_logger(), "Validation success");
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus Validate::on_cancelled()
    {
      RCLCPP_INFO(node_->get_logger(), "Validation cancelled");
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus Validate::on_aborted()
    {
      RCLCPP_INFO(node_->get_logger(), "Validation aborted");
      return BT::NodeStatus::FAILURE;
    }

    void Validate::on_feedback(
      const std::shared_ptr<const weld_interfaces::action::Validate::Feedback> feedback)
    {
      RCLCPP_INFO(node_->get_logger(), "Validation completion: %f", feedback->completion);
    }
  } // namespace weld
}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<jigless_planner::bottom_actions::weld::Validate>(
          name, "validate_joint", config);
      };
  factory.registerBuilder<jigless_planner::bottom_actions::weld::Validate>(
    "Validate", builder);
}
