#include <string>
#include <iostream>

#include "trabajo_final_plansys/behavior_tree_nodes/AttendVisitors.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace trabajo_final_plansys
{

AttendVisitors::AttendVisitors(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
AttendVisitors::halt()
{
  std::cout << "AttendVisitors halt" << std::endl;
}

BT::NodeStatus
AttendVisitors::tick()
{
  // Get person to attend and node to inform about it
  std::string person, location;
  getInput<std::string>("person", person);
  getInput<std::string>("location", location);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  if (!config().blackboard->get("node", node)) {
    RCLCPP_ERROR(node->get_logger(),
    "Failed to get 'node' from the blackboard");
  }

  if (counter_++ < 10) {
    RCLCPP_INFO(node->get_logger(), "ATTENDING VISTOR %s AT %s ...", person.c_str(), location.c_str());
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    RCLCPP_INFO(node->get_logger(), "VISTOR %s ATTENDED AT %s", person.c_str(), location.c_str());
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace trabajo_final_plansys

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trabajo_final_plansys::AttendVisitors>("AttendVisitors");
}