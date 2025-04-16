#include <string>
#include <iostream>

#include "jigless-planner/behavior_tree_nodes/bottom_nodes/Weld.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace jigless_planner::bottom_actions
{
  namespace weld
  {
    Weld::Weld(
      const std::string & xml_tag_name,
      const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf), counter_(0), duration_(10)
    {
    }

    void
    Weld::halt()
    {
      std::cout << "Weld halt" << std::endl;
    }

    BT::NodeStatus
    Weld::tick()
    {
      std::string joint;
      getInput("joint", joint);
      if (counter_ == 0) {
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "Welding " << joint << " tick " << ++counter_ << std::endl;
        return BT::NodeStatus::RUNNING;
      }
      
      auto elapsed_time = std::chrono::steady_clock::now() - start_time_;
      if (elapsed_time < duration_) {
        std::cout << "Weld " << joint << " tick " << ++counter_ << std::endl;
        return BT::NodeStatus::RUNNING;
      } else {
        counter_ = 0;
        return BT::NodeStatus::SUCCESS;
      }
    }
  }
}  // namespace plansys2_bt_example

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<jigless_planner::bottom_actions::weld::Weld>("Weld");
}
