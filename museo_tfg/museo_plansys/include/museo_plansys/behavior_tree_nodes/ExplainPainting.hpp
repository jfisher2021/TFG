// Copyright 2025 Intelligent Robotics Lab
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

#ifndef MUSEO_PLANSYS__BEHAVIOR_TREE_NODES__EXPLAINPAINTING_HPP_
#define MUSEO_PLANSYS__BEHAVIOR_TREE_NODES__EXPLAINPAINTING_HPP_

// Este archivo se mantiene como marcador de posición 
// pero la implementación real está en ExplainPainting.cpp

#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

namespace museo_plansys
{

class ExplainPainting : public BT::ActionNodeBase
{
public:
  explicit ExplainPainting(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts(){
    return BT::PortsList({
      BT::InputPort<std::string>("robot"),
      BT::InputPort<std::string>("painting"),
      BT::OutputPort<std::string>("explanation")
    });
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace museo_plansys

#endif  // MUSEO_PLANSYS__BEHAVIOR_TREE_NODES__EXPLAINPAINTING_HPP_
