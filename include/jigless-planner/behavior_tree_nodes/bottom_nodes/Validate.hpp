#ifndef JIGLESSPLANNER_VALIDATE_HPP
#define JIGLESSPLANNER_VALIDATE_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_planner::bottom_actions
{
  namespace weld
  {
    class Validate : public BT::ActionNodeBase
    {
    public:
      explicit Validate(
        const std::string & xml_tag_name,
        const BT::NodeConfiguration & conf);

      void halt();
      BT::NodeStatus tick();

      static BT::PortsList providedPorts()
      {
        return BT::PortsList({
          BT::InputPort<std::string>("joint")
        });
      }

    private:
      int counter_;
    };
  }
}  // namespace jigless_planner::bottom_actions

#endif  // JIGLESSPLANNER_VALIDATE_HPP
