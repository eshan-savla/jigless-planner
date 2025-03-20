cmake_minimum_required(VERSION 3.8)
project(jigless-planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)
find_package(plansys2_pddl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_bt_actions REQUIRED)
find_package(jigless_planner_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


set(dependencies
    rclcpp
    rclcpp_action
    plansys2_msgs
    plansys2_domain_expert
    plansys2_executor
    plansys2_planner
    plansys2_problem_expert
    plansys2_pddl_parser
    ament_index_cpp
    plansys2_bt_actions
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

# add_executable(transit_action_node src/transit_action_node.cpp)
# ament_target_dependencies(transit_action_node ${dependencies})

# add_executable(weld_action_node src/weld_action_node.cpp)
# ament_target_dependencies(weld_action_node ${dependencies})

# add_executable(validate_action_node src/validate_action_node.cpp)
# ament_target_dependencies(validate_action_node ${dependencies})

add_executable(execute_bottom_node src/ExecuteBottom.cpp)
ament_target_dependencies(execute_bottom_node ${dependencies} rclcpp_components jigless_planner_interfaces)

add_library(jigless_planner_transit_bt_node SHARED src/behavior_tree_nodes/bottom_nodes/Transit.cpp)
list(APPEND plugin_libs jigless_planner_transit_bt_node)

add_library(jigless_planner_weld_bt_node SHARED src/behavior_tree_nodes/bottom_nodes/Weld.cpp)
list(APPEND plugin_libs jigless_planner_weld_bt_node)

add_library(jigless_planner_validate_bt_node SHARED src/behavior_tree_nodes/bottom_nodes/Validate.cpp)
list(APPEND plugin_libs jigless_planner_validate_bt_node)

add_library(jigless_planner_command_bt_node SHARED src/behavior_tree_nodes/top_nodes/Command.cpp)
list(APPEND plugin_libs jigless_planner_command_bt_node)

add_library(jigless_planner_moverobot_bt_node SHARED src/behavior_tree_nodes/top_nodes/MoveRobot.cpp)
list(APPEND plugin_libs jigless_planner_moverobot_bt_node)

add_library(jigless_planner_setstatus_bt_node SHARED src/behavior_tree_nodes/top_nodes/SetStatus.cpp)
list(APPEND plugin_libs jigless_planner_setstatus_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


add_executable(bottom_controller_node src/BottomControllerNode.cpp)
ament_target_dependencies(bottom_controller_node ${dependencies} rclcpp_components rclcpp_lifecycle lifecycle_msgs jigless_planner_interfaces)

add_executable(top_controller_node src/TopControllerNode.cpp)
ament_target_dependencies(top_controller_node ${dependencies} rclcpp_components lifecycle_msgs jigless_planner_interfaces)

install (DIRECTORY launch pddl behavior_trees_xml config DESTINATION share/${PROJECT_NAME})

install(TARGETS
  jigless_planner_transit_bt_node
  jigless_planner_weld_bt_node
  jigless_planner_validate_bt_node
  jigless_planner_command_bt_node
  jigless_planner_moverobot_bt_node
  jigless_planner_setstatus_bt_node
  bottom_controller_node
  top_controller_node
  execute_bottom_node
  # ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/add_problem_client.py
  DESTINATION lib/${PROJECT_NAME})

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
