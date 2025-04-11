// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <iostream>

#include "trabajo_final_plansys/behavior_tree_nodes/ApproachObject.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace trabajo_final_plansys
{

ApproachObject::ApproachObject(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
}

void
ApproachObject::halt()
{
  std::cout << "ApproachObject halt" << std::endl;
}

BT::NodeStatus
ApproachObject::tick()
{
  // Get object to approach and node to inform about it
  std::string object;
  getInput<std::string>("object", object);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  if (!config().blackboard->get("node", node)) {
    RCLCPP_ERROR(node->get_logger(),
    "Failed to get 'node' from the blackboard");
  }

  if (counter_++ < 5) {
    RCLCPP_INFO(node->get_logger(), "APPROACHING OBJECT %s...", object.c_str());
    return BT::NodeStatus::RUNNING;

  } else {
    counter_ = 0;
    RCLCPP_INFO(node->get_logger(), "OBJECT %s APPROACHED", object.c_str());
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace trabajo_final_plansys

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trabajo_final_plansys::ApproachObject>("ApproachObject");
}
