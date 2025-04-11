#ifndef TRABAJO_FINAL_PLANSYS__BEHAVIOR_TREE_NODES__SOLVERUBIK_HPP_
#define TRABAJO_FINAL_PLANSYS__BEHAVIOR_TREE_NODES__SOLVERUBIK_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

namespace trabajo_final_plansys
{

class SolveRubik : public BT::ActionNodeBase 
{
public:
  explicit SolveRubik(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("location")
    };
  }

private:
  typedef enum
  {
    CRUZ_BLANCA,
    ESQUINA_BLANCA,
    SEGUNDA_CAPA,
    LINEAS_COMPLETAS,
    CRUZ_AMARILLA,
    LADOS_AMARILLOS,
    ESQUINAS_AMARILLAS,
    CUBO_COMPLETO
  } CubeState;

  CubeState state_;
  int counter_;
  bool done_;
};

}  // namespace trabajo_final_plansys

#endif  // TRABAJO_FINAL_PLANSYS__BEHAVIOR_TREE_NODES__SOLVERUBIK_HPP_
