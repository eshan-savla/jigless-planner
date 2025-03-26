#include <string>
#include <iostream>

#include "jigless-planner/behavior_tree_nodes/top_nodes/SetCommandable.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace jigless_planner::top_actions
{
  SetCommandable::SetCommandable(
      const std::string & xml_tag_name,
      const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
  {
  }

  void
  SetCommandable::halt()
  {
      std::cout << "SetCommandable halt" << std::endl;
  }

  BT::NodeStatus
  SetCommandable::tick()
  {
      std::cout << "SetCommandable tick " << counter_ << std::endl;

      if (counter_++ < 10) {
      return BT::NodeStatus::RUNNING;
      } else {
      counter_ = 0;
      return BT::NodeStatus::SUCCESS;
      }
  }
}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<jigless_planner::top_actions::SetCommandable>("SetCommandable");
}
