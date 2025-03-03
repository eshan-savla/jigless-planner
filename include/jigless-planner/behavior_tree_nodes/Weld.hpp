#ifndef JIGLESSPLANNER_WELD_HPP
#define JIGLESSPLANNER_WELD_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_planner_weld
{

class Weld : public BT::ActionNodeBase
{
public:
  explicit Weld(
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

#endif  // JIGLESSPLANNER_WELD_HPP
