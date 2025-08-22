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
#include <sys/stat.h>

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
    bool go_ffmpeg = false; // cambiar a true para usar ffmpeg (servicio local)
    if (go_ffmpeg) {
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
          // Intentar reproducir el archivo generado localmente (/tmp/output.wav o /tmp/output.ogg)
          const std::string wav_path = "/tmp/output.wav";
          const std::string ogg_path = "/tmp/output.ogg";
          struct stat buffer;
          std::string play_cmd;
          auto which_exists = [](const char *cmd){ std::string which = std::string("/bin/sh -c 'which ") + cmd + " > /dev/null 2>&1'"; return std::system(which.c_str()) == 0; };

          if (stat(wav_path.c_str(), &buffer) == 0) {
            if (which_exists("paplay")) {
              play_cmd = std::string("paplay \"") + wav_path + "\"";
            } else if (which_exists("aplay")) {
              play_cmd = std::string("aplay \"") + wav_path + "\"";
            } else if (which_exists("ffplay")) {
              play_cmd = std::string("ffplay -nodisp -autoexit -loglevel quiet \"") + wav_path + "\"";
            }
          } else if (stat(ogg_path.c_str(), &buffer) == 0) {
            if (which_exists("gst-launch-1.0")) {
              play_cmd = std::string("gst-launch-1.0 playbin uri=file://") + ogg_path;
            } else if (which_exists("ogg123")) {
              play_cmd = std::string("ogg123 -q \"") + ogg_path + "\"";
            } else if (which_exists("ffplay")) {
              play_cmd = std::string("ffplay -nodisp -autoexit -loglevel quiet \"") + ogg_path + "\"";
            }
          }

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
    else {
      // Llamar al cliente Python para que use Google TTS (mismo flujo que ejecutas manualmente)
      auto shell_quote = [](const std::string &s) {
        std::string r = "'";
        for (char c : s) {
          if (c == '\'') r += "'\\''";
          else r.push_back(c);
        }
        r += "'";
        return r;
      };

      std::cout << "📢 Ejecutando cliente Python TTS con texto: " << text << std::endl;
      const std::string script = "/usr/bin/python3 /home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/my_python_pkg/my_python_pkg/src/tts_client.py ";
      std::string cmd = script + shell_quote(text);
      int rc = std::system(cmd.c_str());
      if (rc == -1) {
        std::cerr << "Error ejecutando el cliente Python TTS" << std::endl;
        return BT::NodeStatus::FAILURE;
      }
      if (WIFEXITED(rc)) {
        int status = WEXITSTATUS(rc);
        if (status == 0) {
          std::cout << "✅ Cliente Python TTS finalizó con éxito" << std::endl;
          return BT::NodeStatus::SUCCESS;
        } else {
          std::cerr << "❌ Cliente Python TTS devolvió código: " << status << std::endl;
          return BT::NodeStatus::FAILURE;
        }
      } else {
        std::cerr << "❌ Cliente Python TTS no terminó correctamente" << std::endl;
        return BT::NodeStatus::FAILURE;
      }
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

