#ifndef JIGLESSPLANNER_WELD_HPP
#define JIGLESSPLANNER_WELD_HPP

#include <string>
#include <chrono>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_planner::bottom_actions
{
  namespace weld 
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
        return BT::PortsList({
          BT::InputPort<std::string>("joint")
        });
      }

    private:
      int counter_;
      std::chrono::steady_clock::time_point start_time_;
      std::chrono::seconds duration_;
    };
  }
}  // namespace jigless_planner::bottom_actions

#endif  // JIGLESSPLANNER_WELD_HPP
