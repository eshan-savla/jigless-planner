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
      const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
    {
    }

    void
    Validate::halt()
    {
      std::cout << "Validate halt" << std::endl;
    }

    BT::NodeStatus
    Validate::tick()
    {
      std::string joint;
      getInput("joint", joint);
      std::cout << "Validate " << joint << " tick " << counter_ << std::endl;

      if (counter_++ < 7) {
        return BT::NodeStatus::RUNNING;
      } else {
        counter_ = 0;
        return BT::NodeStatus::SUCCESS;
      }
    }
  } // namespace weld
}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<jigless_planner::bottom_actions::weld::Validate>("Validate");
}
