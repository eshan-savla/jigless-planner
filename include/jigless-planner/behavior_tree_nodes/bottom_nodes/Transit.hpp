#ifndef JIGLESSPLANNER_TRANSIT_HPP
#define JIGLESSPLANNER_TRANSIT_HPP

#include <string>
#include <unordered_map>
#include <chrono>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <plansys2_problem_expert/ProblemExpertClient.hpp>
#include <plansys2_bt_actions/BTActionNode.hpp>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "transit_interfaces/action/generate_transit.hpp"

namespace jigless_planner::bottom_actions
{
  namespace transit
  {
    class Transit : public plansys2::BtActionNode<transit_interfaces::action::GenerateTransit>
    {
      public:
        explicit Transit(
          const std::string & xml_tag_name,
          const std::string & action_name,
          const BT::NodeConfiguration & conf);
        
        BT::NodeStatus on_tick() override;
        BT::NodeStatus on_success() override;
        BT::NodeStatus on_cancelled() override;
        BT::NodeStatus on_aborted() override;
        void on_feedback(const std::shared_ptr<const 
          transit_interfaces::action::GenerateTransit::Feedback> feedback) override;
        
        static BT::PortsList providedPorts()
        {
          return BT::PortsList({
            BT::InputPort<std::string>("joint1"),
            BT::InputPort<std::string>("joint2"),
            BT::InputPort<std::string>("package_name"),
            BT::InputPort<std::string>("map_file"),
          });
        }
      protected:
        void cancel_goal() override;

      private:
        bool new_joint = true;
        std::string from_joint_, to_joint_;
        std::vector<std::string> workpiece1_, workpiece2_;
        std::unordered_map<std::string, std::vector<std::string>> joint_workpieces_, workpiece_fuses_;
        std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;

        void createMap(const std::string & file_path);
        void updateStatus();
    };
  }
}  // namespace jigless_planner::bottom_actions

#endif  // JIGLESSPLANNER_TRANSIT_HPP
