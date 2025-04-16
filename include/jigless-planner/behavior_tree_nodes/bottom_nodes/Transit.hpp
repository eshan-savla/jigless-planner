#ifndef JIGLESSPLANNER_TRANSIT_HPP
#define JIGLESSPLANNER_TRANSIT_HPP

#include <string>
#include <unordered_map>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace jigless_planner::bottom_actions
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
        return BT::PortsList({
          BT::InputPort<std::string>("joint1"),
          BT::InputPort<std::string>("joint2"),
          BT::InputPort<std::string>("package_name"),
          BT::InputPort<std::string>("map_file"),
        });
      }

    private:
      int counter_;
      std::unordered_map<std::string, std::vector<std::string>> joint_workpieces_, workpiece_fuses_;
      std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;

      std::chrono::steady_clock::time_point start_time_;
      std::chrono::seconds duration_;
      void createMap(const std::string & file_path);
      void updateStatus();
    };
  }
}  // namespace jigless_planner::bottom_actions

#endif  // JIGLESSPLANNER_TRANSIT_HPP
