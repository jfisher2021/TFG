#ifndef TRABAJO_FINAL_PLANSYS__BEHAVIOR_TREE_NODES__ATTENDVISITORS_HPP_
#define TRABAJO_FINAL_PLANSYS__BEHAVIOR_TREE_NODES__ATTENDVISITORS_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace trabajo_final_plansys
{

class AttendVisitors : public BT::ActionNodeBase
{
public:
  explicit AttendVisitors(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("person"),
      BT::InputPort<std::string>("location")
    };
  }

private:
  int counter_;

};

}  // namespace trabajo_final_plansys

#endif  // TRABAJO_FINAL_PLANSYS__BEHAVIOR_TREE_NODES__ATTENDVISITORS_HPP_
