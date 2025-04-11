// Copyright 2021 Intelligent Robotics Lab
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
#include <utility>

#include "trabajo_final_plansys/behavior_tree_nodes/IsPerson.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/behavior_tree.h"

namespace trabajo_final_plansys
{

using namespace std::chrono_literals;
using std::placeholders::_1;

IsPerson::IsPerson(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf), gen(rd())
{
}

BT::NodeStatus
IsPerson::tick()
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  if (!config().blackboard->get("node", node)) {
    RCLCPP_ERROR(node->get_logger(),
    "Failed to get 'node' from the blackboard");
  }

  std::uniform_int_distribution<int> dist(1, 100);
  int rand_num = dist(gen);

  // End with desired probability
  if (rand_num <= SUCCESS_PROB) {
    RCLCPP_INFO(node->get_logger(), "PERSON FOUND");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(node->get_logger(), "NO PERSON FOUND");
  return BT::NodeStatus::FAILURE;
}

}  // namespace

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trabajo_final_plansys::IsPerson>("IsPerson");
}
