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
      const std::string & action_name,
      const BT::NodeConfiguration & conf)
    : plansys2::BtActionNode<transit_interfaces::action::GenerateTransit>(xml_tag_name, action_name,
      conf)
    {
      rclcpp::Node::SharedPtr node;
      config().blackboard->get("node", node);

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
        RCLCPP_INFO(node_->get_logger(), "Failed to load YAML file: %s", file_path.c_str());
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

    BT::NodeStatus Transit::on_tick()
    {
      std::string joint1, joint2;
      getInput<std::string>("joint1", joint1);
      getInput<std::string>("joint2", joint2);
      if (joint1 != from_joint_ || joint2 != to_joint_) {
        updateStatus();
        from_joint_ = joint1;
        to_joint_ = joint2;
        if(joint_workpieces_.find(to_joint_) == joint_workpieces_.end()) {
          RCLCPP_ERROR(node_->get_logger(), "Joint %s not found in map", to_joint_.c_str());
          return BT::NodeStatus::FAILURE;
        }
        workpiece1_ = {joint_workpieces_.find(to_joint_)->second[0]};
        workpiece2_ = {joint_workpieces_.find(to_joint_)->second[1]};
        std::vector<std::string> pieces = joint_workpieces_.find(to_joint_)->second;
        auto it = pieces.begin();
        while (it != pieces.end()){
          std::list<std::string> fuses;
          if (workpiece_fuses_.find(*it) != workpiece_fuses_.end()) {
            fuses.insert(fuses.end(), workpiece_fuses_[*it].begin(), workpiece_fuses_[*it].end());
          }
          if (*it == workpiece1_[0]) {
            workpiece1_.reserve(fuses.size() + workpiece1_.size());
            auto it1 = fuses.begin();
            while (it1 != fuses.end()) {
              auto it2 = std::find(workpiece2_.begin(), workpiece2_.end(), *it1);
              auto it3 = std::find(pieces.begin(), pieces.end(), *it1);
              if (it2 != workpiece2_.end()) {
                workpiece2_.erase(it2);
                pieces.erase(it3);
              }
              workpiece1_.emplace_back(*it1);
              ++it1;
            }
          }
          else {
            workpiece2_.reserve(fuses.size() + workpiece2_.size());
            workpiece2_.insert(workpiece2_.end(), fuses.begin(), fuses.end());
          }
          it = pieces.erase(it);
        }
      }
      goal_.from_joint = from_joint_;
      goal_.to_joint = to_joint_;
      goal_.workpiece1 = workpiece1_;
      goal_.workpiece2 = workpiece2_;
      return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus Transit::on_success()
    {
      RCLCPP_INFO(node_->get_logger(), "Transit success");
      return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus Transit::on_cancelled()
    {
      RCLCPP_INFO(node_->get_logger(), "Transit cancelled");
      // std::string joint1;
      // getInput<std::string>("joint1", joint1);
      // // Reset joint orientation as action effect and predicate undefined in this case
      // RCLCPP_INFO(node_->get_logger(), "Resetting joint orientation to %s", joint1.c_str());
      // problem_expert_->addPredicate(plansys2::Predicate("(joint_orientation " + joint1 + ")"));
      return BT::NodeStatus::SUCCESS;
    }

    void Transit::cancel_goal()
    {
      plansys2::BtActionNode<transit_interfaces::action::GenerateTransit>::cancel_goal();
      RCLCPP_INFO(node_->get_logger(), "Transit goal cancelled");
      // std::string joint1;
      // getInput<std::string>("joint1", joint1);
      // // Reset joint orientation as action effect and predicate undefined in this case
      // RCLCPP_INFO(node_->get_logger(), "Resetting joint orientation to %s", joint1.c_str());
      // problem_expert_->addPredicate(plansys2::Predicate("(joint_orientation " + joint1 + ")"));
    }
    
    BT::NodeStatus Transit::on_aborted()
    {
      RCLCPP_INFO(node_->get_logger(), "Transit aborted");
      // std::string joint1;
      // getInput<std::string>("joint1", joint1);
      // // Reset joint orientation as action effect and predicate undefined in this case
      // RCLCPP_INFO(node_->get_logger(), "Resetting joint orientation to %s", joint1.c_str());
      // problem_expert_->addPredicate(plansys2::Predicate("(joint_orientation " + joint1 + ")"));
      return BT::NodeStatus::FAILURE;
    }

    void Transit::on_feedback(
      const std::shared_ptr<const transit_interfaces::action::GenerateTransit::Feedback> feedback)
    {
      RCLCPP_INFO(node_->get_logger(), "Transit completion: %f", feedback->completion);
    }

  } // namespace transit
}  // namespace jigless_planner::bottom_actions

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string &name, const BT::NodeConfiguration &config) {
    return std::make_unique<jigless_planner::bottom_actions::transit::Transit>(name,
      "generate_transit", config);
    };
  factory.registerBuilder<jigless_planner::bottom_actions::transit::Transit>(
    "Transit", builder);
}
