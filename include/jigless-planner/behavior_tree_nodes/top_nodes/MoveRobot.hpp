#ifndef JIGLESSPLANNER_MOVEROBOT_HPP
#define JIGLESSPLANNER_MOVEROBOT_HPP

#include <string>
#include <chrono>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_planner::top_actions
{

  class MoveRobot : public BT::ActionNodeBase
  {
  public:
    explicit MoveRobot(
      const std::string & xml_tag_name,
      const BT::NodeConfiguration & conf);

    void halt();
    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
      return BT::PortsList({});
    }

  private:
    int counter_;
    std::chrono::steady_clock::time_point start_time_;
    std::chrono::seconds duration_;
  };

}  // namespace_ jigless_planner

#endif  // JIGLESSPLANNER_MOVEROBOT_HPP
