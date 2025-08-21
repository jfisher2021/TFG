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

#include <string>
#include <iostream>
#include <fstream>
#include <memory>

#include "museo_plansys/behavior_tree_nodes/ExplainPainting.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "rclcpp/rclcpp.hpp"

namespace museo_plansys
{

ExplainPainting::ExplainPainting(
const std::string & xml_tag_name,
const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf)
{
    // node_ is not needed anymore since we won't use rclcpp logging
}

void
ExplainPainting::halt()
{
    std::cout << "ExplainPainting halt" << std::endl;
}

BT::NodeStatus ExplainPainting::tick()
{
    std::cout << "[ExplainPainting] tick" << std::endl;

    std::string robot;
    std::string painting;

    if (!getInput("robot", robot)) {
            std::cout << "[ExplainPainting] Missing 'robot' input" << std::endl;
            return BT::NodeStatus::FAILURE;
    }

    if (!getInput("painting", painting)) {
            std::cout << "[ExplainPainting] Missing 'painting' input" << std::endl;
            return BT::NodeStatus::FAILURE;
    }

    std::cout << "[ExplainPainting] Explaining with " << robot << " the painting " << painting << std::endl;

    // Cargar la explicación desde un archivo
    std::fstream my_file;
    std::string explanation;
    my_file.open("/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/explicacion_respuestas/" + painting + ".txt", std::ios::in);
    if (!my_file) {
            std::cout << "[ExplainPainting] No se pudo abrir el archivo de explicación para " << painting << std::endl;
            return BT::NodeStatus::FAILURE;
    } else {
            std::string line;
            while (std::getline(my_file, line)) {
                explanation += line + "\n";
            }
    }
    my_file.close();

    std::cout << "Explicacion cargada:\n" << explanation << std::endl;

    // Pasar la explicación al siguiente nodo
    setOutput("explanation", explanation);

    return BT::NodeStatus::SUCCESS;
}

static BT::PortsList providedPorts()
{
    return BT::PortsList({
            BT::InputPort<std::string>("robot"),
            BT::InputPort<std::string>("painting"),
            BT::OutputPort<std::string>("explanation")
    });
}

};

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<museo_plansys::ExplainPainting>("ExplainPainting");
}

