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
  : rclcpp::Node("library_controller_node"),  state_(SELECTING)
  {
  }

  bool init()
  {
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();

    drawings = {"monalisa", "nocheestrellada", "elgrito", "guernica", "dibejo"};
    goals_ = {"(and explained_painting monalisa_ws)", "(and visited tiago monalisa_ws)", "(and visited tiago elgrito_ws)"};
    RCLCPP_INFO(get_logger(), "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA!");

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
    case SELECTING:
    {
      // End when no goals left
      if (goals_.empty()) {
        state_ = ENDING;
        break;
      }
      RCLCPP_INFO(get_logger(), "SELECTINGGGGGGGGGGGGGGGGGGGGG");

      // actual_goal_ = goals_.back();
      
      // problem_expert_->setGoal(plansys2::Goal("(and " + actual_goal_ + " )"));
      problem_expert_->setGoal(plansys2::Goal("(and (explained_painting monalisa) (explained_painting elgrito) (explained_painting dibejo) (explained_painting guernica) (explained_painting nocheestrellada))"));

      RCLCPP_INFO(get_logger(), "2222222222222222SELLLLLLLLLLLLLLLLLL");


      auto domain = domain_expert_->getDomain();
      auto problem = problem_expert_->getProblem();
      auto plan = planner_client_->getPlan(domain, problem);
      // for (const auto & plan_item : plan.value().items) {
      //     std::cout << plan_item.time << ":\t" << plan_item.action << "\t[" <<
      //       plan_item.duration << "]" << std::endl;
      // }
      RCLCPP_INFO(get_logger(), "333333333333333sellllllllllllllll!!");

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

      // // Check what type of goal is
      // if (actual_goal_.find("rubik_solved") != std::string::npos){
      //   RCLCPP_INFO(get_logger(), "GOAL RUBIK... ");
      //   state_ = MOVE;
      // }
  
      // if (actual_goal_.find("book_found") != std::string::npos){
      //   RCLCPP_INFO(get_logger(), "GOAL BOOK... ");
      //   state_ = EXPLAIN;
      // }

      // if (actual_goal_.find("person_attended") != std::string::npos){
      //   RCLCPP_INFO(get_logger(),  "GOAL ATTEND... ");
      //   state_ = RECHARGE;
      // }

      state_ = MOVE;
      RCLCPP_INFO(get_logger(), "444444444444444sellllllllllllllllll!!");

      break;
    }
    case MOVE:
    {
      // RCLCPP_INFO(get_logger(), "11111111111111111111111111MOVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE!!");

      if (!executor_client_->execute_and_check_plan()) {  // Plan finished
        RCLCPP_INFO(get_logger(), "22222222222222222222MOVEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE!!");

        switch (executor_client_->getResult().value().result) {
          case plansys2_msgs::action::ExecutePlan::Result::SUCCESS:
            RCLCPP_INFO(get_logger(), "RUBIK SOLVED !!");
            // goals_.pop_back();
            break;

          case plansys2_msgs::action::ExecutePlan::Result::PREEMPT:
            RCLCPP_INFO(get_logger(), "Plan preempted");
            break;

          case plansys2_msgs::action::ExecutePlan::Result::FAILURE:
            RCLCPP_ERROR(get_logger(), "RUBIK NOT SOLVED");
            break;
        }
        // problem_expert_->clearGoal();
        state_ = SELECTING;
      }
      break;
    }
    case EXPLAIN:
    {
      if (!executor_client_->execute_and_check_plan()) {  // Plan finished
        switch (executor_client_->getResult().value().result) {
          case plansys2_msgs::action::ExecutePlan::Result::SUCCESS:
            RCLCPP_INFO(get_logger(), "PERSON ATTENDED !!");
            goals_.pop_back();
            break;

          case plansys2_msgs::action::ExecutePlan::Result::PREEMPT:
            RCLCPP_INFO(get_logger(), "Plan preempted");
            break;

          case plansys2_msgs::action::ExecutePlan::Result::FAILURE:
            RCLCPP_ERROR(get_logger(), "NO PERSON ATTENDED");
          break;
        }
        problem_expert_->clearGoal();
        state_ = SELECTING;
      }
      break;
    }
    case RECHARGE:
    {
      if (!executor_client_->execute_and_check_plan()) {  // Plan finished
        switch (executor_client_->getResult().value().result) {
          case plansys2_msgs::action::ExecutePlan::Result::SUCCESS:
            RCLCPP_INFO(get_logger(), "BOOK FOUND !!");
            goals_.pop_back();
            break;

          case plansys2_msgs::action::ExecutePlan::Result::PREEMPT:
            RCLCPP_INFO(get_logger(), "Plan preempted");
            break;

          case plansys2_msgs::action::ExecutePlan::Result::FAILURE:
            RCLCPP_ERROR(get_logger(), "BOOK NOT FOUND");

            // If book is not where it was supposed to,
            // move it to a new shelf and restart execution
            if (drawings.empty()) {
              RCLCPP_INFO(get_logger(), "NO MORE SHELVES TO SEARCH");
              goals_.pop_back();

            } else { // Change shelf where book is supposed to be at
              problem_expert_->removePredicate(plansys2::Predicate("(book_at book_1 " + drawings.back() + ")"));
              drawings.pop_back();
              problem_expert_->addPredicate(plansys2::Predicate("(book_at book_1 " + drawings.back() + ")"));
            }
            break;
        }
        problem_expert_->clearGoal();
        state_ = SELECTING;
      }
      break;
    }
    case ENDING:
      RCLCPP_INFO(get_logger(), "All goals achieved...");
      break;

    default:
      break;
    }
  }

private:
  typedef enum {SELECTING, MOVE, EXPLAIN, RECHARGE, ENDING} StateType;
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
