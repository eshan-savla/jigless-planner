#include <string>
#include <iostream>

#include "jigless-planner/behavior_tree_nodes/bottom_nodes/Transit.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace jigless_planner::bottom_actions
{
  namespace transit
  {
    Transit::Transit(
      const std::string & xml_tag_name,
      const BT::NodeConfiguration & conf)
    : BT::ActionNodeBase(xml_tag_name, conf), counter_(0), duration_(5)
    {
      problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
      std::string package_name, map_file;
      if (!getInput("package_name", package_name)) {
        throw BT::RuntimeError("Missing required input [package_name]");
      }
      if (!getInput("map_file", map_file)) {
        throw BT::RuntimeError("Missing required input [map_file]");
      }
      std::string file_path = ament_index_cpp::get_package_share_directory(package_name) + "/" + map_file;
      createMap(file_path);
    }
    
    void Transit::createMap(const std::string & file_path)
    {
      YAML::Node yaml_node = YAML::LoadFile(file_path);
      if (yaml_node.IsNull()) {
        std::cerr << "Failed to load YAML file: " << file_path << std::endl;
        return;
      }
      if(yaml_node["workpieces"]) {
        for (const auto & item : yaml_node["workpieces"]) {
          joint_workpieces_[item.first.as<std::string>()] = item.second.as<std::vector<std::string>>();
        }
      }
    }

    void Transit::updateStatus()
    {
      std::vector<plansys2::Predicate> predicates = problem_expert_->getPredicates();
      for (std::size_t i = 0; i < predicates.size(); ++i) {
        if (predicates[i].name == "welded"){
          std::vector<std::string> workpieces = joint_workpieces_[predicates[i].parameters[0].name];
          for (const auto & workpiece : workpieces) {
            std::unordered_set<std::string> fuses_set(workpiece_fuses_[workpiece].begin(),
              workpiece_fuses_[workpiece].end());
            fuses_set.insert(workpieces.begin(), workpieces.end());
            fuses_set.erase(workpiece);
            std::vector<std::string> fuses(fuses_set.begin(), fuses_set.end());
            workpiece_fuses_[workpiece].reserve(fuses.size());            
            workpiece_fuses_[workpiece] = fuses;
          }
        }
      }
    }

    void
    Transit::halt()
    {
      std::cout << "Transit halt" << std::endl;
    }

    BT::NodeStatus
    Transit::tick()
    {
      std::string joint1;
      getInput<std::string>("joint1", joint1);
      std::string joint2;
      getInput<std::string>("joint2", joint2);
      std::stringstream out_ss;
      if (counter_++ == 0 ) {
        start_time_ = std::chrono::steady_clock::now();
        updateStatus();
        out_ss << "Transiting to " << joint2 << " from " << joint1 << std::endl;
        if(joint_workpieces_.find(joint2) == joint_workpieces_.end()) {
          std::cerr << "Joint " << joint2 << " not found in map";
          return BT::NodeStatus::FAILURE;
        }
        std::vector<std::string> pieces = joint_workpieces_.find(joint2)->second;
        auto it = pieces.begin();
        while (it != pieces.end()){
          std::list<std::string> fuses;
          if (workpiece_fuses_.find(*it) != workpiece_fuses_.end()) {
            fuses.insert(fuses.end(), workpiece_fuses_[*it].begin(), workpiece_fuses_[*it].end());
          }
          out_ss << std::endl << "Moving workpieces: " << *it;
          for (const auto & fuse : fuses) {
            out_ss << " + " << fuse;
            auto it2 = std::find(pieces.begin(), pieces.end(), fuse);
            if (it2 != pieces.end())
              pieces.erase(it2);
          }
          it = pieces.erase(it);
        }
        std::cout << out_ss.str() << std::endl;
        return BT::NodeStatus::RUNNING;
      }
      auto elapsed_time = std::chrono::steady_clock::now() - start_time_;
      if (elapsed_time < duration_) {
        std::cout << "Transit tick " << ++counter_ << std::endl;
        return BT::NodeStatus::RUNNING;
      } else {
        counter_ = 0;
        return BT::NodeStatus::SUCCESS;
      }
    }
  } // namespace transit
}  // namespace jigless_planner::bottom_actions

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<jigless_planner::bottom_actions::transit::Transit>("Transit");
}
