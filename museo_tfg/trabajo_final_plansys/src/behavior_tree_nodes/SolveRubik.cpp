#include <string>
#include <iostream>

#include "trabajo_final_plansys/behavior_tree_nodes/SolveRubik.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace trabajo_final_plansys
{

SolveRubik::SolveRubik(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf),
  state_(CRUZ_BLANCA), counter_(0), done_(false)
{
}

void
SolveRubik::halt()
{
  std::cout << "SolveRubik halt" << std::endl;
}

BT::NodeStatus
SolveRubik::tick()
{
  std::string location;
  getInput<std::string>("location", location);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  if (!config().blackboard->get("node", node)) {
    RCLCPP_ERROR(node->get_logger(),
    "Failed to get 'node' from the blackboard");
  }

  if (done_) {
    if (counter_++ < 10) {
      RCLCPP_INFO(node->get_logger(), "SOLVED THE RUBIK'S CUBE AT %s", location.c_str());
      return BT::NodeStatus::RUNNING;

    } else {
      counter_ = 0;
      done_ = false;
      return BT::NodeStatus::SUCCESS;
    }
  }

  srand(time(NULL));
  float detected_;
  detected_ = static_cast<float>(std::rand())/ static_cast<float>(RAND_MAX);

  if (detected_ < 0.9){
    switch (state_)
    {
    case CRUZ_BLANCA:
      RCLCPP_INFO(node->get_logger(), "MADE THE WHITE CROSS");
      state_ = ESQUINA_BLANCA;
      break;
    case ESQUINA_BLANCA:
      RCLCPP_INFO(node->get_logger(), "PLACED WHITE CORNERS");
      state_ = SEGUNDA_CAPA;
      break;
    case SEGUNDA_CAPA:
      RCLCPP_INFO(node->get_logger(), "COMPLETED SECOND LAYER");
      state_ = CRUZ_AMARILLA;
      break;
    case CRUZ_AMARILLA:
      RCLCPP_INFO(node->get_logger(), "MADE THE YELLOW CROSS");
      state_ = ESQUINAS_AMARILLAS;
      break;
    case ESQUINAS_AMARILLAS:
      RCLCPP_INFO(node->get_logger(), "PLACED YELLOW CORNERS");
      state_ = LADOS_AMARILLOS;
      break;
    case LADOS_AMARILLOS:
      RCLCPP_INFO(node->get_logger(), "YELLOW SIDES DONE");
      state_ = CUBO_COMPLETO;
      break;

    case CUBO_COMPLETO:
      done_ = true;
      break;

    default:
      RCLCPP_INFO(node->get_logger(), "Unknown state");
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node->get_logger(), "I dont know how to solve Rubik's Cube :(");
  return BT::NodeStatus::FAILURE;
}

}  // namespace trabajo_final_plansys

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<trabajo_final_plansys::SolveRubik>("SolveRubik");
}