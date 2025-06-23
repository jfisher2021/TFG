#include <memory>
#include <random>

#include "plansys2_pddl_parser/Utils.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class Controller : public rclcpp::Node
{
public:
  Controller()
  : rclcpp::Node("library_controller_node"),  state_(PLANNING)
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    drawings = {
      "monalisa", "nocheestrellada", "elgrito", "dibejo", "guernica", "la_joven_de_la_perla", "las_meninas",
      "el_3_de_mayo_de_1808", "el_jardin_de_las_delicias", "las_tres_gracias", "la_rendicion_de_breda",
      "el_nacimiento_de_venus", "la_creacion_de_adan", "la_ultima_cena", "la_libertad_guiando_al_pueblo",
      "el_hijo_del_hombre", "american_gothic", "la_persistencia_de_la_memoria", "la_ronda_de_noche",
      "impresion_sol_naciente", "banistas_en_asnieres", "saturno_devorando_a_su_hijo", "whistlers_mother",
      "la_gran_ola_de_kanagawa", "el_beso", "autorretrato_con_collar_de_espinas", "el_carnaval_del_arlequin",
      "nighthawks", "retrato_de_adele_bloch_bauer_i", "campbells_soup_cans", "composition_viii"
    };
    goals_ = {"(and explained_painting monalisa_ws)", "(and visited tiago monalisa_ws)", "(and visited tiago elgrito_ws)"};

    init_knowledge();

    return true;
  }

  void init_knowledge()
  {
    problem_expert_->addInstance(plansys2::Instance{"home", "location"});
    for (auto shelf : drawings) {
      problem_expert_->addInstance(plansys2::Instance{shelf, "location"});
    }

    problem_expert_->addInstance(plansys2::Instance{"tiago", "robot"});
    problem_expert_->addInstance(plansys2::Instance{"pers_1", "person"});

    problem_expert_->addPredicate(plansys2::Predicate("(initial_state tiago)"));
    problem_expert_->addPredicate(plansys2::Predicate("(robot_at tiago home)"));

    problem_expert_->addPredicate(plansys2::Predicate("(charger_at home)"));
    problem_expert_->addFunction(plansys2::Function("(= (battery tiago) 100)"));

    // problem_expert_->addPredicate(plansys2::Predicate("(person_at pers_1 door)"));

  }

  void step()
  {
    switch (state_)
    {
    case PLANNING:
    {
      // End when no goals left
      if (goals_.empty()) {
        state_ = ENDING;
        break;
      }
      RCLCPP_INFO(get_logger(), "PLANNINGGGGGGGGGGGGGGGGGGGG!!");



      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);
      // for (const auto & plan_item : plan.value().items) {
      //     std::cout << plan_item.time << ":\t" << plan_item.action << "\t[" <<
      //       plan_item.duration << "]" << std::endl;
      // }
      RCLCPP_INFO(get_logger(), "WE HAVE PLANNNNNN!!");

      // if there is no plan, retry setting it
      if (!plan.has_value()) {
        RCLCPP_ERROR_STREAM(get_logger(),
              "Could not find plan for " + parser::pddl::toString(problem_expert_->getGoal()));
        break;
      }

      // if the planned plan does not start, retry
      if (!executor_client_->start_plan_execution(plan.value())) {
        RCLCPP_ERROR(get_logger(), "Error starting plan");
        break;
      }

      state_ = DOING;
      RCLCPP_INFO(get_logger(), "LESTTTT DOOO ITTTT!!");

      break;
    }
    case DOING:
    {
      // RCLCPP_INFO(get_logger(), "11111111111111111111111111MOVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE!!");

      if (!executor_client_->execute_and_check_plan()) {  // Plan finished
        RCLCPP_INFO(get_logger(), "IM MOOVIINGGGG!!");

        switch (executor_client_->getResult().value().result) {
          case plansys2_msgs::action::ExecutePlan::Result::SUCCESS:
            RCLCPP_INFO(get_logger(), "TOUR FINISHEDDD !!");
            state_ = ENDING;
            // goals_.pop_back();
            break;

          case plansys2_msgs::action::ExecutePlan::Result::PREEMPT:
            RCLCPP_INFO(get_logger(), "Plan preempted");
            state_ = PLANNING;
            break;

          case plansys2_msgs::action::ExecutePlan::Result::FAILURE:
            RCLCPP_ERROR(get_logger(), "RUBIK NOT SOLVED");
            state_ = PLANNING;
            break;
        }
        // problem_expert_->clearGoal();
        
      }
      break;
    }
    case ENDING:
      static bool finished = false;
      if (!finished) {
        RCLCPP_INFO(get_logger(), "All goals achieved...");
        finished = true;
      }
      break;

    default:
      break;
    }
  }

private:
  typedef enum {PLANNING, DOING, ENDING} StateType;
  StateType state_;

  std::shared_ptr<plansys2::DomainExpertClient> domain_expert_;
  std::shared_ptr<plansys2::PlannerClient> planner_client_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_expert_;
  std::shared_ptr<plansys2::ExecutorClient> executor_client_;

  std::vector<std::string> drawings;
  std::vector<std::string> goals_;
  std::string actual_goal_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Controller>();

  if (!node->init()) {
    return 0;
  }

  rclcpp::Rate rate(5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
