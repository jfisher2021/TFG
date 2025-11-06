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

#include <memory>
#include <iostream>
#include <cstdlib>
#include <ctime>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "my_interfaces/srv/text_to_speech.hpp"

using namespace std::chrono_literals;

class RechargeAction : public plansys2::ActionExecutorClient
{
public:
  RechargeAction()
  : plansys2::ActionExecutorClient("recharge", 1s)
  {    
    // Crear cliente del servicio TTS
    node = rclcpp::Node::make_shared("recharge_client");
    tts_client_ = node->create_client<my_interfaces::srv::TextToSpeech>("tts_service");
  }

  rclcpp::Node::SharedPtr node;


private:
  void do_work()
  {
    
    std::string arg_robot = get_arguments()[0];
    std::string arg_to = get_arguments()[1];
    
    std::cout << "Recharging with " << arg_robot
              << " in " << arg_to << std::endl;

    // Llamar al servicio TTS
    while (!tts_client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
        finish(false, 0.0, "recharge failed");
      }
      RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }

    // Hacer la llamada al servicio TTS
    auto request = std::make_shared<my_interfaces::srv::TextToSpeech::Request>();
    request->text = "recargando la bateria para seguir con la mision.";
    auto result_future = tts_client_->async_send_request(request);
    

    // Esperar la respuesta del servicio
    if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node->get_logger(), "service call failed :(");
      tts_client_->remove_pending_request(result_future);
      finish(false, 0.0, "recharge failed");
    }

    
    // Obtener el resultado de la llamada al servicio
    auto result = result_future.get();
    if (result->success) {
      RCLCPP_INFO(get_logger(), "✅ TTS completado con éxito");
      finish(true, 1.0, "recharge completed");
    } else {
      RCLCPP_ERROR(get_logger(), "❌ Error en TTS: %s", result->debug.c_str());
      finish(false, 0.0, "TTS error");
    }
    
    finish(true, 1.0, "recharge completed");
    RCLCPP_INFO(get_logger(), "RECARGADO!!!");
    
  }

  rclcpp::Client<my_interfaces::srv::TextToSpeech>::SharedPtr tts_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RechargeAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "recharge"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
