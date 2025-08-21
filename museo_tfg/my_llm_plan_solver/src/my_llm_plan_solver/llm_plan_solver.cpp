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

namespace plansys2
{

LLMPlanSolver::LLMPlanSolver()
{
}

std::optional<std::filesystem::path>
LLMPlanSolver::create_folders(const std::string & node_namespace)
{
  auto output_dir = lc_node_->get_parameter(output_dir_parameter_name_).value_to_string();
  std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBOutput directory: " << output_dir << std::endl;

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

  if (!lc_node_->has_parameter(arguments_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(arguments_parameter_name_, "");
  }
  if (!lc_node_->has_parameter(output_dir_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(
      output_dir_parameter_name_, std::filesystem::temp_directory_path());
  }
}

// Función auxiliar para obtener la explicación de un cuadro
std::string obtener_explicacion(const std::string& cuadro) {
  // Aquí iría tu llamada al LLM, por ejemplo:
  RCLCPP_INFO(
    rclcpp::get_logger("LLMPlanSolver"), "Obteniendo explicación para el cuadro: %s", cuadro.c_str());
  std::string command = "ssh dedalo.tsc.urjc.es 'python3 /home/jfisher/tfg/preguntas_sobre_csv.py " + cuadro + "'";
  std::string result;
  char buffer[128];
  FILE* pipe = popen(command.c_str(), "r");
  if (!pipe) return "";
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    result += buffer;
  }
  pclose(pipe);
  return result;
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
  RCLCPP_DEBUG(
    lc_node_->get_logger(), "Writing planning results to %s.", output_dir.string().c_str());
  RCLCPP_INFO(
    lc_node_->get_logger(), "Writing planning results to %s.", output_dir.string().c_str());

  // // COPY_EXAMPLE AND RETURNS THE PLAN IN /tmp/plan TIENE QUE ESTAR ACTIVADO EL SSH DEL SERVIDOR
  // std::string command = "ssh dedalo.tsc.urjc.es '/home/jfisher/miniconda3/bin/python /home/jfisher/tfg/tfg_ollama/copy_example.py'";
  // std::string result;
  // // system("ls"); // Clear the terminal screen for better visibility

  // char buffer[128];

  // FILE* pipe = popen(command.c_str(), "r");
  // if (!pipe) {
  //   std::cerr << "Error abriendo el pipe\n";
  //   return {};
  // }

  // while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
  //   result += buffer;
  // }

  // pclose(pipe);
  // std::cout << "Salida remota:\n" << result << std::endl;
  // std::ofstream plan_out("/tmp/plan");
  // if (plan_out.is_open()) {
  //   plan_out << result;
  //   plan_out.close();
  // } else {
  //   std::cerr << "No se pudo abrir /tmp/plan para escribir la salida remota\n";
  //   return {};
  // }
  const auto plan_file_path = output_dir / std::filesystem::path("plan");

  return parse_plan_result(plan_file_path.string());
}

std::optional<plansys2_msgs::msg::Plan>
LLMPlanSolver::parse_plan_result(const std::string & plan_path)
{
  std::string line;
  std::ifstream plan_file(plan_path);
  std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAaa Parsing plan file: " << plan_path << std::endl;
  bool solution = false;

  plansys2_msgs::msg::Plan plan;

  // Mapa para guardar las explicaciones
  std::unordered_map<std::string, std::future<std::string>> explicaciones_futures;

  if (plan_file.is_open()) {
    while (getline(plan_file, line)) {
      std::cout << "XXXXXXXXXXXXXXXXXXLine: " << line << std::endl;
      if (!solution) {
        if (line.find("start_welcome") != std::string::npos) {
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
          std::cout << "UUUUUUUUUUUUUUUUUUUUUUUUUUUUParsed item: " << item.action << " at time " << item.time << " with duration " << item.duration << std::endl;
          solution = true;
        }
      } else if (line.front() != ';') {
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
        // if (item.action.find("explain") != std::string::npos) {
        //   // Extrae solo el nombre del cuadro (por ejemplo: "monalisa")
        //   size_t last_space = item.action.find_last_of(' ');
        //   size_t last_paren = item.action.find(')', last_space);
        //   std::string cuadro = item.action.substr(last_space + 1, last_paren - last_space - 1);
        //   // Lanza el hilo y guarda el future
        //   explicaciones_futures[cuadro] = std::async(std::launch::async, obtener_explicacion, cuadro);
        // }

        plan.items.push_back(item);
        std::cout << "UUUUUUUUUUUUUUUUUUUUUUUUUUUUParsed item: " << item.action << " at time " << item.time << " with duration " << item.duration << std::endl;
      }
    }
    plan_file.close();
  }

  // // Espera a que todas las explicaciones estén listas y guárdalas donde quieras
  // for (auto& [cuadro, fut] : explicaciones_futures) {
  //   std::string explicacion = fut.get();
  //   // Puedes guardar la explicación en disco o en memoria, por ejemplo:
  //   std::ofstream out("/tmp/explicacion_" + cuadro);
  //   out << explicacion;
  //   out.close();
  // }

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
