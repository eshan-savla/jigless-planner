#ifndef JIGLESSPLANNER_COMMAND_HPP
#define JIGLESSPLANNER_COMMAND_HPP

#include <string>
#include <chrono>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_planner::top_actions
{
  class Command : public BT::ActionNodeBase
  {
  public:
    explicit Command(
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

}  // namespace_ jigless_planner::top_actions

#endif  // JIGLESSPLANNER_COMMAND_HPP
