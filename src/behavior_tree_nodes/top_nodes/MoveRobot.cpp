#include <string>
#include <iostream>

#include "jigless-planner/behavior_tree_nodes/top_nodes/MoveRobot.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace jigless_planner::top_actions
{
  MoveRobot::MoveRobot(
      const std::string & xml_tag_name,
      const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf), counter_(0), duration_(5)
  {
  }

  void
  MoveRobot::halt()
  {
      std::cout << "MoveRobot halt" << std::endl;
  }

  BT::NodeStatus
  MoveRobot::tick()
  {
    std::string joint;
    getInput("joint", joint);
    if (counter_ == 0) {
        start_time_ = std::chrono::steady_clock::now();
        std::cout << "Moving " << joint << " tick " << ++counter_ << std::endl;
        return BT::NodeStatus::RUNNING;
    }
    auto elapsed_time = std::chrono::steady_clock::now() - start_time_;
    if (elapsed_time < duration_) {
        std::cout << "MoveRobot " << joint << " tick " << ++counter_ << std::endl;
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
  factory.registerNodeType<jigless_planner::top_actions::MoveRobot>("MoveRobot");
}
