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
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


#include "plansys2_executor/ActionExecutorClient.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


class ExplainAction : public plansys2::ActionExecutorClient
{
public:
  ExplainAction()
  : plansys2::ActionExecutorClient("explain_painting", 1s)
  {
    success_ = 0.0;
    retry = 0;
    // Create a subscription to the LLM request topic
  }
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state)
  {
    pub_llm_ = this->create_publisher<std_msgs::msg::String>(
      "llm_response", 10);

    // sub_llm_ = this->create_subscription<std_msgs::msg::String>(
    //   "llm_request", rclcpp::SensorDataQoS(),
    //   std::bind(&ExplainAction::scan_callback, this, std::placeholders::_1));

    return ActionExecutorClient::on_activate(state);
  }


private:
  void do_work()
  {
    srand(time(NULL));
    success_ = static_cast<float>(std::rand())/ static_cast<float>(RAND_MAX);
    // std::cout << success_ << std::endl;
    // RCLCPP_INFO(get_logger(), "Detected: %f\n", success_);
    std::string arg_robot = get_arguments()[0];
    std::string drawing = get_arguments()[1];
    // std::cout << "\r\e[K" << std::flush;
    std::cout << "Explaining with " << arg_robot
              << " drawing " << drawing << std::endl;

    // Here you can process the message and send a response if needed
    std_msgs::msg::String response;
    response.data = "Response to: " + drawing;
    pub_llm_->publish(response);
    if (success_ > 0.7) {
      std::cout << "FOUND!!!" << std::endl;
      std::string python_arg =  "Explicame el siguiente cuadro: " + drawing;
      std::string command = "ssh dedalo.tsc.urjc.es 'python3 /home/jfisher/tfg/preguntas_sobre_csv.py " + python_arg + "'";
      std::string result;
      // system("ls"); // Clear the terminal screen for better visibility

      char buffer[128];

      FILE* pipe = popen(command.c_str(), "r");
      if (!pipe) {
        std::cerr << "Error abriendo el pipe\n";
        finish(false, 0.0, "explain_painting failed");
      }

      while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
      }

      pclose(pipe);
      std::cout << "Salida remota:\n" << result << std::endl;
      finish(true, 1.0, "explain_painting completed");
      success_ = 0.0;
    } else {
      if (retry >= 5) {
        finish(false, 0.0, "explain_painting failed");
        retry = 0;
        success_ = 0.0;
      } else {
        send_feedback(success_, "explain_painting running");
        retry++;
      }
    }

  }

  // void
  // scan_callback(const std_msgs::msg::String::SharedPtr msg)
  // {
  //   RCLCPP_WARN(get_logger(), "🚨 Callback triggered");
  //   RCLCPP_INFO(get_logger(), "Received LLM request: '%s'", msg->data.c_str());
  //   // Here you can process the message and send a response if needed
  //   std_msgs::msg::String response;
  //   response.data = "Response to: " + msg->data;
  //   pub_llm_->publish(response);
  // }
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_llm_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_llm_;

  float success_;
  int retry;
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
