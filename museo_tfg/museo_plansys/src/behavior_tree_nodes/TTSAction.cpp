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
#include <memory>

#include "museo_plansys/behavior_tree_nodes/TTSAction.hpp"
#include "behaviortree_cpp/behavior_tree.h"

#include "rclcpp/rclcpp.hpp"
#include "my_interfaces/srv/text_to_speech.hpp"

namespace museo_plansys
{

TTSAction::TTSAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BT::ActionNodeBase(xml_tag_name, conf)
  {
    node_ = rclcpp::Node::make_shared("tts_bt_node");
    
    tts_client_ = node_->create_client<my_interfaces::srv::TextToSpeech>("tts_service");
    
    // Esperar a que el servicio esté disponible
    while (!tts_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        std::cerr << "Interrumpido mientras esperaba al servicio TTS" << std::endl;
        return;
      }
      std::cout << "Esperando al servicio TTS..." << std::endl;
    }
    
    std::cout << "Servicio TTS disponible" << std::endl;
  }

  void
  TTSAction::halt()
  {
      std::cout << "ExplainPainting halt" << std::endl;
  }
  BT::NodeStatus TTSAction::tick() 
  {
    std::cout << "TTSAction tick" << std::endl;
    
    std::string text;
    if (!getInput("text", text)) {
      std::cerr << "Missing 'text' input" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    
    // Crear solicitud
    auto request = std::make_shared<my_interfaces::srv::TextToSpeech::Request>();
    request->text = text;
    
    std::cout << "📢 Enviando solicitud TTS" << std::endl;
    
    // Enviar solicitud (forma síncrona para simplicidad en el BT)
    auto future = tts_client_->async_send_request(request);
    
    // Esperar resultado
    if (rclcpp::spin_until_future_complete(node_, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future.get();
      if (result->success) {
        std::cout << "✅ TTS completado con éxito" << std::endl;
        return BT::NodeStatus::SUCCESS;
      } else {
        std::cerr << "❌ Error en TTS: " << result->debug << std::endl;
        return BT::NodeStatus::FAILURE;
      }
    } else {
      std::cerr << "❌ Error al llamar al servicio TTS" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({
      BT::InputPort<std::string>("text")
    });
  }

}  // namespace museo_plansys

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<museo_plansys::TTSAction>("TTSAction");
}

