#ifndef JIGLESSPLANNER_TRANSIT_HPP
#define JIGLESSPLANNER_TRANSIT_HPP

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_bottom_planner
{
  namespace transit
  {
    class Transit : public BT::ActionNodeBase
    {
    public:
      explicit Transit(
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
  }
}  // namespace jigless_bottom_planner

#endif  // JIGLESSPLANNER_TRANSIT_HPP
