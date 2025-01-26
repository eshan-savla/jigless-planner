#ifndef JIGLESSPLANNER_MOVE_WORKPIECE_HPP
#define JIGLESSPLANNER_MOVE_WORKPIECE_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_planner_transit
{

class MoveWorkpiece : public BT::ActionNodeBase
{
public:
  explicit MoveWorkpiece(
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

}  // namespace jigless_planner_transit

#endif  // JIGLESSPLANNER_MOVE_WORKPIECE_HPP
