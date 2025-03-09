#ifndef JIGLESSPLANNER_MOVEROBOT_HPP
#define JIGLESSPLANNER_MOVEROBOT_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_top_planner
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
  };

}  // namespace_ jigless_top_planner

#endif  // JIGLESSPLANNER_MOVEROBOT_HPP
