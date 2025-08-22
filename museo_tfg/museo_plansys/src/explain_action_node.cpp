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
  ExplainAction()
  : plansys2::ActionExecutorClient("explain_painting", 1s)
  {
    gtts_client_ = this->create_client<my_interfaces::srv::TextToSpeech>("tts_service");
  }

  // rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  // on_activate(const rclcpp_lifecycle::State & state)
  // {
  //   say_pub_ = this->create_publisher<std_msgs::msg::String>(
  //     "say_text", 10);

  //   // sub_llm_ = this->create_subscription<std_msgs::msg::String>(
  //   //   "llm_request", rclcpp::SensorDataQoS(),
  //   //   std::bind(&ExplainAction::scan_callback, this, std::placeholders::_1));

  //   return ActionExecutorClient::on_activate(state);
  // }


private:
  void do_work()
  {
    
    std::string arg_robot = get_arguments()[0];
    std::string drawing = get_arguments()[1];
    // std::cout << "\r\e[K" << std::flush;
    std::cout << "FOUND!!! Explaining with " << arg_robot
              << " drawing " << drawing << std::endl;

    // Here you can process the message and send a response if needed
    std_msgs::msg::String msg;

    // std::string python_arg =  "Explicame el siguiente cuadro: " + drawing;
    // std::string command = "ssh dedalo.tsc.urjc.es 'python3 /home/jfisher/tfg/tfg_ollama/preguntas_sobre_csv.py " + python_arg + "'";
    // std::string result;
    // system("ls"); // Clear the terminal screen for better visibility

    // char buffer[128];

    // FILE* pipe = popen(command.c_str(), "r");
    // if (!pipe) {
    //   std::cerr << "Error abriendo el pipe\n";
    //   finish(false, 0.0, "explain_painting failed");
    // }

    // while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    //   result += buffer;
    // }

    // pclose(pipe);

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

    // 🔁 Esperar resultado (bloqueante, al estilo HNI)
    try {
      auto result = future.get();
      RCLCPP_INFO(get_logger(), "📢 Respuesta de TTS recibida");
      if (result->success) {
        RCLCPP_INFO(get_logger(), "✅ TTS leído con éxito");
      } else {
        RCLCPP_WARN(get_logger(), "⚠️ TTS falló: %s", result->debug.c_str());
      }

      // ✅ Si llegamos aquí, todo OK
      RCLCPP_INFO(get_logger(), "✅ Finalizando acción...");
      finish(true, 1.0, "explain_painting completed");

    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "❌ Excepción esperando respuesta de TTS: %s", e.what());
      finish(false, 0.0, "TTS exception");
    }
    


    // msg.data = ch;
    // say_pub_->publish(msg);

    
    finish(true, 1.0, "explain_painting completed");
    

  }

  // void
  // scan_callback(const std_msgs::msg::String::SharedPtr msg)
  // {
  //   RCLCPP_WARN(get_logger(), "🚨 Callback triggered");
  //   RCLCPP_INFO(get_logger(), "Received LLM request: '%s'", msg->data.c_str());
  //   // Here you can process the message and send a response if needed
  //   std_msgs::msg::String response;
  //   response.data = "Response to: " + msg->data;
  //   say_pub_->publish(response);
  // }
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_llm_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr say_pub_;
  rclcpp::Client<my_interfaces::srv::TextToSpeech>::SharedPtr gtts_client_;


};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExplainAction>();

  node->set_parameter(rclcpp::Parameter("action_name", "explain_painting"));
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);


  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
