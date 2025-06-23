cmake_minimum_required(VERSION 3.8)
project(museo_plansys)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(my_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    rclcpp_action
    geometry_msgs
    tf2_geometry_msgs
    nav2_msgs
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
    my_interfaces
)


include_directories(include)

# ADD NODES OF THE BEHAVIOR TREE

#! REAL MOVE NODE OF NAV2
# add_library(plansys2_move_bt_node SHARED src/behavior_tree_nodes/Move.cpp)
# list(APPEND plugin_libs plansys2_move_bt_node)


# ADD THE DEPENDENCIES
foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# ADD THE EXECUTABLES

add_executable(tts_client_node src/tts_client_node.cpp)
ament_target_dependencies(tts_client_node ${dependencies})

add_executable(move_fake_action_node src/move_fake_action_node.cpp)
ament_target_dependencies(move_fake_action_node ${dependencies})

add_executable(explain_action_node src/explain_action_node.cpp)
ament_target_dependencies(explain_action_node ${dependencies})

add_executable(recharge_action_node src/recharge_action_node.cpp)
ament_target_dependencies(recharge_action_node ${dependencies})

add_executable(welcome_action_node src/welcome_action_node.cpp)
ament_target_dependencies(welcome_action_node ${dependencies})

add_executable(library_controller_node src/library_controller_node.cpp)
ament_target_dependencies(library_controller_node ${dependencies})


install(DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  move_fake_action_node
  explain_action_node
  tts_client_node
  welcome_action_node
  recharge_action_node
  library_controller_node
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()
