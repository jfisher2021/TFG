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
#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "my_interfaces/srv/text_to_speech.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class ExplainAction : public plansys2::ActionExecutorClient
{
public:
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
    std::string arg_robot = get_arguments()[0];
    std::string drawing = get_arguments()[1];
    // std::cout << "\r\e[K" << std::flush;
    std::cout << "EXPLICANDO CON " << arg_robot
              << " EL CUADRO " << drawing << std::endl;

    std_msgs::msg::String msg;

    // Obtener el directorio share del paquete
    std::string pkg_share;
    try {
      pkg_share = ament_index_cpp::get_package_share_directory("museo_plansys");
      RCLCPP_DEBUG(get_logger(), "Directorio share del paquete: %s", pkg_share.c_str());
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "No se ha podido encontrar el directorio share del paquete: %s", e.what());
      finish(false, 0.0, "Package directory no encontrado");
      return;
    }

    // Construir ruta al archivo de explicación
    std::filesystem::path file_path = std::filesystem::path(pkg_share) / 
                                      "explicacion_respuestas" / 
                                      (drawing + ".txt");

    RCLCPP_DEBUG(get_logger(), "Leyendo explicación desde: %s", file_path.string().c_str());

    // Verificar que el archivo existe
    if (!std::filesystem::exists(file_path)) {
      RCLCPP_ERROR(get_logger(), "Archivo de explicación no encontrado: %s", file_path.string().c_str());
      finish(false, 0.0, "Archivo de explicación no encontrado");
      return;
    }

    // Leer el contenido del archivo
    std::ifstream my_file(file_path.string(), std::ios::in);
    if (!my_file.is_open()) {
      RCLCPP_ERROR(get_logger(), "No se pudo abrir el archivo: %s", file_path.string().c_str());
      finish(false, 0.0, "No se pudo abrir el archivo de explicación");
      return;
    }

    std::string explanation_text;
    std::string line;
    while (std::getline(my_file, line)) {
      explanation_text += line + "\n";
    }
    my_file.close();

    if (explanation_text.empty()) {
      RCLCPP_WARN(get_logger(), "Archivo de explicación vacío: %s", drawing.c_str());
      finish(false, 0.0, "Archivo de explicación vacío");
      return;
    }

    // Crear solicitud TTS
    auto request = std::make_shared<my_interfaces::srv::TextToSpeech::Request>();
    request->text = explanation_text;

    RCLCPP_INFO(get_logger(), "📢 Enviando solicitud TTS");

    // Enviar solicitud asíncrona
    auto future = gtts_client_->async_send_request(request);
    RCLCPP_INFO(get_logger(), "⏳ Esperando respuesta de TTS...");

    // Esperar resultado con timeout
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(120)) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(get_logger(), "✅ CUADRO EXPLICADO CON ÉXITO");
        finish(true, 1.0, "explain_painting completed");
      } else {
        RCLCPP_ERROR(get_logger(), "❌ Error en TTS: %s", result->debug.c_str());
        finish(false, 0.0, "TTS error");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "❌ Error al llamar al servicio TTS");
      finish(false, 0.0, "TTS call error");
    }
  }
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
