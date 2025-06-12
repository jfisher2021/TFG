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

using namespace std::chrono_literals;

class RechargeAction : public plansys2::ActionExecutorClient
{
public:
  RechargeAction()
  : plansys2::ActionExecutorClient("recharge", 1s)
  {
    success_ = 0.0;
    retry = 0;
  }

private:
  void do_work()
  {
    srand(time(NULL));
    success_ = static_cast<float>(std::rand())/ static_cast<float>(RAND_MAX);
    // std::cout << success_ << std::endl;
    // RCLCPP_INFO(get_logger(), "Detected: %f\n", success_);
    std::string arg_robot = get_arguments()[0];
    std::string arg_to = get_arguments()[1];
    // std::cout << "\r\e[K" << std::flush;
    std::cout << "Recharging with " << arg_robot
              << " in " << arg_to
              << " ... ["
              << std::min(100.0, success_ * 100.0)
              << "% of accuracy]  " << std::endl;


    if (success_ > 0.2) {
      std::cout << "FOUND!!!" << std::endl;
      finish(true, 1.0, "recharge completed");
      success_ = 0.0;
      std::cout << std::endl;
    } else {
      if (retry >= 10) {
        finish(false, 0.0, "recharge failed");
        std::cout << std::endl;
        retry = 0;
        success_ = 0.0;
      } else {
        send_feedback(success_, "recharge running");
        retry++;
      }
    }

  }

  float success_;
  int retry;
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
