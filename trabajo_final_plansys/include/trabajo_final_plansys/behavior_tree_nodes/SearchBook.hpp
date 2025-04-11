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

#ifndef TRABAJO_FINAL_PLANSYS__BEHAVIOR_TREE_NODES__ISPERSON_HPP_
#define TRABAJO_FINAL_PLANSYS__BEHAVIOR_TREE_NODES__ISPERSON_HPP_

#include <string>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace trabajo_final_plansys
{

class SearchBook : public BT::ConditionNode
{
public:
  explicit SearchBook(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick();
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("book"),
      BT::InputPort<std::string>("location")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;

  std::random_device rd;
  std::mt19937 gen;

  const int SUCCESS_PROB = 20;
};

}  // namespace

#endif  //
