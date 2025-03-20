#include <string>
#include <iostream>

#include "jigless-planner/behavior_tree_nodes/bottom_nodes/Transit.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace jigless_planner::bottom_actions
{
  namespace transit
  {
    Transit::Transit(
      const std::string & xml_tag_name,
      const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
    {
    }

    void
    Transit::halt()
    {
      std::cout << "Transit halt" << std::endl;
    }

    BT::NodeStatus
    Transit::tick()
    {
      std::cout << "Transit tick " << counter_ << std::endl;

      if (counter_++ < 5) {
        return BT::NodeStatus::RUNNING;
      } else {
        counter_ = 0;
        return BT::NodeStatus::SUCCESS;
      }
    }
  } // namespace transit
}  // namespace jigless_planner::bottom_actions

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<jigless_planner::bottom_actions::transit::Transit>("Transit");
}
