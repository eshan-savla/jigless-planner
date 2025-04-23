#ifndef JIGLESSPLANNER_VALIDATE_HPP
#define JIGLESSPLANNER_VALIDATE_HPP

#include <string>
#include <chrono>
#include <plansys2_bt_actions/BTActionNode.hpp>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "weld_interfaces/action/validate.hpp"

namespace jigless_planner::bottom_actions
{
  namespace weld
  {
    class Validate : public plansys2::BtActionNode<weld_interfaces::action::Validate>
    {
    public:
      explicit Validate(
        const std::string & xml_tag_name,
        const std::string & action_name,
        const BT::NodeConfiguration & conf);

      BT::NodeStatus on_tick() override;
      BT::NodeStatus on_success() override;
      BT::NodeStatus on_cancelled() override;
      BT::NodeStatus on_aborted() override;
      void on_feedback(const std::shared_ptr<const 
        weld_interfaces::action::Validate::Feedback> feedback) override;
      
      static BT::PortsList providedPorts()
      {
        return BT::PortsList({
          BT::InputPort<std::string>("joint")
        });
      }
    };
  }
}  // namespace jigless_planner::bottom_actions

#endif  // JIGLESSPLANNER_VALIDATE_HPP
