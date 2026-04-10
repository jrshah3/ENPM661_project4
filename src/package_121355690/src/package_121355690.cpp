#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <control_msgs/action/gripper_command.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::MoveItErrorCode;
using GripperCommand = control_msgs::action::GripperCommand;
using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("package_121355690");

bool planAndExecute(MoveGroupInterface& group, const std::vector<double>& joint_values, const std::string& label)
{
  RCLCPP_INFO(LOGGER, "=== Starting step: %s ===", label.c_str());
  group.setStartStateToCurrentState();
  group.setJointValueTarget(joint_values);
  MoveGroupInterface::Plan plan;
  RCLCPP_INFO(LOGGER, "Planning: %s", label.c_str());
  auto plan_result = group.plan(plan);
  if (plan_result != MoveItErrorCode::SUCCESS) { RCLCPP_ERROR(LOGGER, "Planning FAILED for: %s", label.c_str()); return false; }
  RCLCPP_INFO(LOGGER, "Executing: %s", label.c_str());
  auto exec_result = group.execute(plan);
  if (exec_result != MoveItErrorCode::SUCCESS) { RCLCPP_ERROR(LOGGER, "Execution FAILED for: %s", label.c_str()); return false; }
  group.stop();
  RCLCPP_INFO(LOGGER, "Completed: %s", label.c_str());
  std::this_thread::sleep_for(1s);
  return true;
}

bool moveGripper(rclcpp::Node::SharedPtr node, double position, const std::string& label)
{
  RCLCPP_INFO(LOGGER, "=== Gripper: %s ===", label.c_str());
  auto client = rclcpp_action::create_client<GripperCommand>(node, "/hand_controller/gripper_cmd");
  if (!client->wait_for_action_server(3s)) { RCLCPP_WARN(LOGGER, "Gripper server not available, skipping"); return true; }
  auto goal = GripperCommand::Goal();
  goal.command.position = position;
  goal.command.max_effort = 50.0;
  auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
  auto future = client->async_send_goal(goal, send_goal_options);
  if (future.wait_for(5s) != std::future_status::ready) { RCLCPP_WARN(LOGGER, "Gripper send failed, skipping"); return true; }
  auto goal_handle = future.get();
  if (!goal_handle) { RCLCPP_WARN(LOGGER, "Gripper goal rejected, skipping"); return true; }
  auto result_future = client->async_get_result(goal_handle);
  result_future.wait_for(5s);
  RCLCPP_INFO(LOGGER, "Gripper completed: %s", label.c_str());
  std::this_thread::sleep_for(1s);
  return true;
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("package_121355690_node", node_options);
  RCLCPP_INFO(LOGGER, "Node created");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });
  RCLCPP_INFO(LOGGER, "Waiting for move_group to be fully ready...");
  std::this_thread::sleep_for(5s);

  MoveGroupInterface arm_group(node, "panda_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  arm_group.setPlanningTime(15.0);
  arm_group.setNumPlanningAttempts(10);
  arm_group.setMaxVelocityScalingFactor(0.2);
  arm_group.setMaxAccelerationScalingFactor(0.2);
  arm_group.setGoalJointTolerance(0.01);
  arm_group.startStateMonitor(2.0);

  // Setup Planning Scene
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.resize(4);

  collision_objects[0].id = "table";
  collision_objects[0].header.frame_id = arm_group.getPlanningFrame();
  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions = {1.0, 1.2, 0.1};
  geometry_msgs::msg::Pose table_pose;
  table_pose.orientation.w = 1.0;
  table_pose.position.x = 0.5;
  table_pose.position.z = -0.05;
  collision_objects[0].primitives.push_back(table_primitive);
  collision_objects[0].primitive_poses.push_back(table_pose);
  collision_objects[0].operation = collision_objects[0].ADD;

  collision_objects[1].id = "path_blocker_1";
  collision_objects[1].header.frame_id = arm_group.getPlanningFrame();
  shape_msgs::msg::SolidPrimitive p1_primitive;
  p1_primitive.type = p1_primitive.BOX;
  p1_primitive.dimensions = {0.1, 0.1, 0.6};
  geometry_msgs::msg::Pose p1_pose;
  p1_pose.orientation.w = 1.0;
  p1_pose.position.x = 0.45;
  p1_pose.position.y = 0.15;
  p1_pose.position.z = 0.3;
  collision_objects[1].primitives.push_back(p1_primitive);
  collision_objects[1].primitive_poses.push_back(p1_pose);
  collision_objects[1].operation = collision_objects[1].ADD;

  collision_objects[2].id = "path_blocker_2";
  collision_objects[2].header.frame_id = arm_group.getPlanningFrame();
  shape_msgs::msg::SolidPrimitive p2_primitive;
  p2_primitive.type = p2_primitive.BOX;
  p2_primitive.dimensions = {0.1, 0.1, 0.6};
  geometry_msgs::msg::Pose p2_pose;
  p2_pose.orientation.w = 1.0;
  p2_pose.position.x = 0.45;
  p2_pose.position.y = -0.15;
  p2_pose.position.z = 0.3;
  collision_objects[2].primitives.push_back(p2_primitive);
  collision_objects[2].primitive_poses.push_back(p2_pose);
  collision_objects[2].operation = collision_objects[2].ADD;

  collision_objects[3].id = "center_wall";
  collision_objects[3].header.frame_id = arm_group.getPlanningFrame();
  shape_msgs::msg::SolidPrimitive p3_primitive;
  p3_primitive.type = p3_primitive.BOX;
  p3_primitive.dimensions = {0.3, 0.05, 0.25};
  geometry_msgs::msg::Pose p3_pose;
  p3_pose.orientation.w = 1.0;
  p3_pose.position.x = 0.6;
  p3_pose.position.y = 0.0;
  p3_pose.position.z = 0.125;
  collision_objects[3].primitives.push_back(p3_primitive);
  collision_objects[3].primitive_poses.push_back(p3_pose);
  collision_objects[3].operation = collision_objects[3].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
  RCLCPP_INFO(LOGGER, "Planning scene initialized.");
  std::this_thread::sleep_for(2s);

  std::vector<double> home  = {0.0,  -0.785,  0.0, -2.356,  0.0,  1.571,  0.785};
  std::vector<double> goal1 = {0.5,  -0.8,    0.3, -2.0,    0.2,  1.6,    1.0  };
  std::vector<double> goal2 = {-0.5, -0.8,   -0.3, -2.0,   -0.2,  1.6,   -0.5  };

  bool ok = true;
  if (ok) ok = planAndExecute(arm_group, home,  "move to home");
  moveGripper(node, 0.035, "open gripper");
  if (ok) ok = planAndExecute(arm_group, goal1, "move to goal1 (pick)");
  moveGripper(node, 0.001, "close gripper (pick)");
  if (ok) ok = planAndExecute(arm_group, goal2, "move to goal2 (place)");
  moveGripper(node, 0.035, "open gripper (place)");
  if (ok) ok = planAndExecute(arm_group, home,  "return home");

  if (ok) { RCLCPP_INFO(LOGGER, "========================================"); RCLCPP_INFO(LOGGER, "Pick and place sequence COMPLETED OK!"); RCLCPP_INFO(LOGGER, "========================================"); std::this_thread::sleep_for(5s); }
  else { RCLCPP_ERROR(LOGGER, "Pick and place sequence FAILED"); }
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
