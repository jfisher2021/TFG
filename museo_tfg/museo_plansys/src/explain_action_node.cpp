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
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "plansys2_executor/ActionExecutorClient.hpp"
#include "my_interfaces/srv/text_to_speech.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class ExplainAction : public plansys2::ActionExecutorClient
{
public:
  std::cout << "QQQQQQQQQQQQQQQQQQQQQQQQQQ:\n" << ch << std::endl;
  ExplainAction()
  : plansys2::ActionExecutorClient("explain_painting", 1s)
  {
    node_ = rclcpp::Node::make_shared("tts_bt_node");
    gtts_client_ = node_->create_client<my_interfaces::srv::TextToSpeech>("tts_service");
  }
  rclcpp::Node::SharedPtr node_;

private:
  void do_work()
  {
    std::cout << "QQQQQQQQQQQQQQQQQQQQQQQQQQ:\n" << ch << std::endl;

    std::string arg_robot = get_arguments()[0];
    std::string drawing = get_arguments()[1];
    // std::cout << "\r\e[K" << std::flush;
    std::cout << "FOUND!!! Explaining with " << arg_robot
              << " drawing " << drawing << std::endl;

    // Here you can process the message and send a response if needed
    std_msgs::msg::String msg;

    std::fstream my_file;
    std::string ch;
    my_file.open("/home/jfisherr/cuarto/2c/plansis/plansys_ws/src/TFG/museo_tfg/museo_plansys/explicacion_respuestas/" + drawing + ".txt", std::ios::in);
    if (!my_file) {
      std::cout << "No such file";
    }
    else {

      std::string line;
      while (std::getline(my_file, line)) {
        ch += line + "\n";
      }

    }
    my_file.close();
    
    std::cout << "Explicacion:\n" << ch << std::endl;
    // Crear solicitud
    auto request = std::make_shared<my_interfaces::srv::TextToSpeech::Request>();
    request->text = ch;

    RCLCPP_INFO(get_logger(), "📢 Enviando solicitud TTS: %s", request->text.c_str());

    // Enviar solicitud
    auto future = gtts_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "⏳ Esperando respuesta de TTS...");

    // Esperar resultado
    if (rclcpp::spin_until_future_complete(node_, future) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future.get();
      if (result->success) {
        std::cout << "✅ TTS completado con éxito" << std::endl;
        finish(true, 1.0, "explain_painting completed");
      } else {
        std::cerr << "❌ Error en TTS: " << result->debug << std::endl;
        finish(false, 0.0, "TTS error");
      }
    } else {
      std::cerr << "❌ Error al llamar al servicio TTS" << std::endl;
      finish(false, 0.0, "TTS call error");
    }
    
    finish(true, 1.0, "explain_painting completed");

  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr say_pub_;
  rclcpp::Client<my_interfaces::srv::TextToSpeech>::SharedPtr gtts_client_;


};

int main(int argc, char ** argv)
{
  std::cout << "QQQQQQQQQQQQQQQQQQQQQQQQQQ:\n" << ch << std::endl;

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExplainAction>();
  
  node->set_parameter(rclcpp::Parameter("action_name", "explain_painting"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
