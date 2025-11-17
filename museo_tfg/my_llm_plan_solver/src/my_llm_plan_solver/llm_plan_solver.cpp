// Copyright 2019 Intelligent Robotics Lab
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


#include <fstream>
#include <thread>
#include <future>
#include <unordered_map>

#include "plansys2_msgs/msg/plan_item.hpp"
#include "my_llm_plan_solver/llm_plan_solver.hpp"
#include <std_msgs/msg/string.hpp>
#include "my_interfaces/srv/text_to_speech.hpp"
#include "my_interfaces/srv/speech_to_text.hpp"
namespace plansys2
{

LLMPlanSolver::LLMPlanSolver()
{
}

std::optional<std::filesystem::path>
LLMPlanSolver::create_folders(const std::string & node_namespace)
{
  auto output_dir = lc_node_->get_parameter(output_dir_parameter_name_).value_to_string();

  // Allow usage of the HOME directory with the `~` character, returning if there is an error.
  const char * home_dir = std::getenv("HOME");
  if (output_dir[0] == '~' && home_dir) {
    output_dir.replace(0, 1, home_dir);
  } else if (!home_dir) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "Invalid use of the ~ character in the path: %s", output_dir.c_str()
    );
    return std::nullopt;
  }

  // Create the necessary folders, returning if there is an error.
  auto output_path = std::filesystem::path(output_dir);
  if (node_namespace != "") {
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        output_path /= p;
      }
    }
    try {
      std::filesystem::create_directories(output_path);
    } catch (std::filesystem::filesystem_error & err) {
      RCLCPP_ERROR(lc_node_->get_logger(), "Error writing directories: %s", err.what());
      return std::nullopt;
    }
  }
  return output_path;
}

void LLMPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  lc_node_ = lc_node;

  arguments_parameter_name_ = plugin_name + ".arguments";
  output_dir_parameter_name_ = plugin_name + ".output_dir";
  python_command_parameter_ = plugin_name + ".python_command";
  python_env_parameter_ = plugin_name + ".python_env";

  if (!lc_node_->has_parameter(arguments_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(arguments_parameter_name_, "");
  }
  if (!lc_node_->has_parameter(output_dir_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(
      output_dir_parameter_name_, std::filesystem::temp_directory_path());
  }

  if (!lc_node_->has_parameter(python_command_parameter_)) {
      lc_node_->declare_parameter<std::string>(python_command_parameter_, "");
    }
    if (!lc_node_->has_parameter(python_env_parameter_)) {
      lc_node_->declare_parameter<std::string>(python_env_parameter_, "");
    }

  // Crear nodo independiente para no bloquear el lifecycle node de plansys2
  nodos_servicios = rclcpp::Node::make_shared("llm_plan_solver_services_node");
  
  // Crear clientes del servicio en el nodo independiente
  tts_client_ = nodos_servicios->create_client<my_interfaces::srv::TextToSpeech>("tts_service");
  stt_client_ = nodos_servicios->create_client<my_interfaces::srv::SpeechToText>("stt_service");
    
  RCLCPP_INFO(lc_node_->get_logger(), "LLM Plan Solver configured with independent service node");
}

std::optional<plansys2_msgs::msg::Plan>
LLMPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace,
  const rclcpp::Duration solver_timeout)
{
  // Set up the folders
  const auto output_dir_maybe = create_folders(node_namespace);
  if (!output_dir_maybe) {
    return {};
  }
  const auto & output_dir = output_dir_maybe.value();
  std::string result;

  std::string spoken_goal = "";
  std::string python_arg;

  std::string python_env = lc_node_->get_parameter(python_env_parameter_).value_to_string();
  std::string python_command = lc_node_->get_parameter(python_command_parameter_).value_to_string();
  std::string command;
  RCLCPP_INFO(lc_node_->get_logger(), "python_env: %s", python_env.c_str());
  RCLCPP_INFO(lc_node_->get_logger(), "python_command: %s", python_command.c_str());

  char buffer[128];

  // TTS 
  if (tts_client_->wait_for_service(std::chrono::seconds(5))) {
    auto tts_request = std::make_shared<my_interfaces::srv::TextToSpeech::Request>();
    tts_request->text = "elige tu goal";
    auto tts_future = tts_client_->async_send_request(tts_request);

    if (rclcpp::spin_until_future_complete(nodos_servicios, tts_future, std::chrono::seconds(15))
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(lc_node_->get_logger(), "TTS service timeout");
    }
  }

  // STT 
  if (stt_client_->wait_for_service(std::chrono::seconds(5))) {
    auto stt_request = std::make_shared<my_interfaces::srv::SpeechToText::Request>();
    auto stt_future = stt_client_->async_send_request(stt_request);

    if (rclcpp::spin_until_future_complete(nodos_servicios, stt_future, std::chrono::seconds(120))
        == rclcpp::FutureReturnCode::SUCCESS) {
      auto stt_result = stt_future.get();
      if (stt_result->success) {
        spoken_goal = stt_result->text;
        RCLCPP_INFO(lc_node_->get_logger(), "Transcripción recibida con éxito");
      } else {
        RCLCPP_ERROR(lc_node_->get_logger(), "Error en STT: %s", stt_result->debug.c_str());
      }
    } else {
      RCLCPP_ERROR(lc_node_->get_logger(), "Timeout al llamar al servicio STT");
    }
  } else {
    RCLCPP_ERROR(lc_node_->get_logger(), "STT service no disponible tras 5s");
  }

  if (spoken_goal.empty()) {
    python_arg = "Explicame el siguiente cuadro: ";
  } else {
    python_arg = "Create a plan for goal: " + spoken_goal;
  }
  RCLCPP_INFO(lc_node_->get_logger(), "%s", python_arg.c_str());
  // std::string command = "ssh dedalo.tsc.urjc.es '/home/jfisher/miniconda3/bin/python /home/jfisher/tfg/tfg_ollama/create_plan.py " + python_arg + "'";
  command = python_env + " " + python_command + " " + python_arg;
  
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) {
    RCLCPP_ERROR(lc_node_->get_logger(), "Failed to execute plan generation command");
    return {};
  }

  // Abrir archivo en modo append para escribir en tiempo real
  std::ofstream plan_out("/tmp/plan", std::ios::out | std::ios::trunc);
  if (!plan_out.is_open()) {
    RCLCPP_ERROR(lc_node_->get_logger(), "Failed to open /tmp/plan for writing");
    pclose(pipe);
    return {};
  }


  setvbuf(pipe, nullptr, _IONBF, 0);
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
    plan_out << buffer;
    plan_out.flush();  
  }

  plan_out.close();
  pclose(pipe);
  
  return parse_plan_result("/tmp/plan");
}

std::optional<plansys2_msgs::msg::Plan>
LLMPlanSolver::parse_plan_result(const std::string & plan_path)
{
  std::string line;
  std::ifstream plan_file(plan_path);
  bool solution = false;

  plansys2_msgs::msg::Plan plan;

  if (plan_file.is_open()) {
    while (getline(plan_file, line)) {

      if (line == "0.000: (start_welcome tiago) [1.000]") {
        plansys2_msgs::msg::PlanItem item;
        size_t colon_pos = line.find(":");
        size_t colon_par = line.find(")");
        size_t colon_bra = line.find("[");

        std::string time = line.substr(0, colon_pos);
        std::string action = line.substr(colon_pos + 2, colon_par - colon_pos - 1);
        std::string duration = line.substr(colon_bra + 1);
        duration.pop_back();

        item.time = std::stof(time);
        item.action = action;
        item.duration = std::stof(duration);
        plan.items.clear(); // No duplicar el plan si el LLM da varias versiones
        plan.items.push_back(item);

        solution = true;
      }
      // solo procesar líneas que tengan el formato esperado ":" ")" "[" "tiago"
      else if (line.find(":") != std::string::npos && line.find(")") != std::string::npos
      && line.find("[") != std::string::npos && line.find("tiago") != std::string::npos) {
        plansys2_msgs::msg::PlanItem item;
        size_t colon_pos = line.find(":");
        size_t colon_par = line.find(")");
        size_t colon_bra = line.find("[");

        std::string time = line.substr(0, colon_pos);
        std::string action = line.substr(colon_pos + 2, colon_par - colon_pos - 1);
        std::string duration = line.substr(colon_bra + 1);
        duration.pop_back();
        item.time = std::stof(time);
        item.action = action;
        item.duration = std::stof(duration);
        plan.items.push_back(item);
      }
    }
    plan_file.close();
  }

  if (solution && !plan.items.empty()) {
    return plan;
  } else {
    return {};
  }
}

bool
LLMPlanSolver::isDomainValid(
  const std::string & domain,
  const std::string & node_namespace)
{
  if (system(nullptr) == 0) {
    return false;
  }

  // Set up the folders
  const auto output_dir_maybe = create_folders(node_namespace);
  if (!output_dir_maybe) {
    return {};
  }
  const auto & output_dir = output_dir_maybe.value();
  RCLCPP_DEBUG(
    lc_node_->get_logger(), "Writing domain validation results to %s.",
    output_dir.string().c_str()
  );

  // Perform domain validation
  const auto domain_file_path = output_dir / std::filesystem::path("check_domain.pddl");
  std::ofstream domain_out(domain_file_path);
  domain_out << domain;
  domain_out.close();

  const auto problem_file_path = output_dir / std::filesystem::path("check_problem.pddl");
  std::ofstream problem_out(problem_file_path);
  problem_out << "(define (problem void) (:domain plansys2))";
  problem_out.close();

  const auto plan_file_path = output_dir / std::filesystem::path("check.out");
  const int status = system(
    ("ros2 run popf popf " +
    domain_file_path.string() + " " + problem_file_path.string() + " > " + plan_file_path.string())
    .c_str());

  if (status == -1) {
    return false;
  }

  std::string line;
  std::ifstream plan_file(plan_file_path);
  bool solution = false;

  if (plan_file && plan_file.is_open()) {
    while (getline(plan_file, line)) {
      if (!solution) {
        if (line.find("Solution Found") != std::string::npos) {
          solution = true;
        }
      }
    }
    plan_file.close();
  }

  return solution;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::LLMPlanSolver, plansys2::PlanSolverBase);
