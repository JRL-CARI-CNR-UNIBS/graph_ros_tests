// ROS and Moveit related libraries
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Graph core libraries
#include <graph_display/graph_display.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <moveit_collision_checker/collision_checkers/parallel_moveit_collision_checker.h>
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("test_path", options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Load logger configuration file
  std::string package_name = "graph_ros_tests";
  std::string package_path = ament_index_cpp::get_package_share_directory(package_name);

  if (package_path.empty())
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Failed to get path for package '" << package_name);
    return 1;
  }

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_path",logger_file);

  // Get the robot description
  std::string param_ns1 = "/"+package_name;
  std::string param_ns2 = param_ns1+"/test_path";
  std::string group_name;
  if(not graph::core::get_param(logger,param_ns2,"group_name",group_name))
    return 1;

  robot_model_loader::RobotModelLoader robot_model_loader(node,"robot_description");
  moveit::core::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  const moveit::core::JointModelGroup* joint_model_group =  kinematic_model->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const moveit::core::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  // Update the planning scene
  graph::display::DisplayPtr display = std::make_shared<graph::display::Display>(planning_scene,group_name,kinematic_model->getLinkModelNames().back());
  kinematic_model->getLinkModelNames();
  rclcpp::sleep_for(std::chrono::seconds(1));

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr ps_client =
    node->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");

  if (!ps_client->wait_for_service(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(node->get_logger(),"Unable to connect to /get_planning_scene");
    return 1;
  }

  auto ps_srv = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  auto result = ps_client->async_send_request(ps_srv);
  if (rclcpp::spin_until_future_complete(node, result)!=rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(),"Call to srv not ok");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(result.get()->scene))
  {
    RCLCPP_ERROR(node->get_logger(),"unable to update planning scene");
    return 1;
  }

  // Set-up planning tools
  int n_threads;
  graph::core::get_param(logger,param_ns2,"parallel_checker_n_threads",n_threads,4);

  double checker_resolution;
  graph::core::get_param(logger,param_ns2,"checker_resolution",checker_resolution,0.01);

  double max_distance;
  graph::core::get_param(logger,param_ns2,"max_distance",max_distance,1.0);

  bool use_kdtree = true;
  graph::core::get_param(logger,param_ns2,"use_kdtree",use_kdtree,true);

  graph::core::CollisionCheckerPtr checker = std::make_shared<graph::ros2::ParallelMoveitCollisionChecker>(planning_scene, group_name, logger, n_threads, checker_resolution);
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  graph::core::PathPtr path;
  graph::core::get_param(logger,param_ns1,"path",path,metrics,checker);

  graph::core::TreePtr tree;
  graph::core::get_param(logger,param_ns1,"tree",tree,metrics,checker);

  path->setTree(tree);

  tree->print_full_tree_ = true;

  RCLCPP_INFO_STREAM(node->get_logger(),"Tree:\n"<<*tree);
  RCLCPP_INFO_STREAM(node->get_logger(),"Path:\n"<<*path);

  rclcpp::sleep_for(std::chrono::milliseconds(100));

  display->clearMarkers();
  display->displayPathAndWaypoints(path);
  display->displayTree(tree,"graph_display",{0.0,0.0,1.0,0.01});

  RCLCPP_INFO(node->get_logger(),"Flip the path");
  path->flip();
  RCLCPP_INFO_STREAM(node->get_logger(),"Flipped path\n "<<*path);

  RCLCPP_INFO(node->get_logger(),"Warp on cloned path");
  double warp_min_conn_length;
  graph::core::get_param(logger,param_ns2,"warp_min_conn_length",warp_min_conn_length,0.1);

  double warp_min_step_size;
  graph::core::get_param(logger,param_ns2,"warp_min_step_size",warp_min_step_size,0.1);

  graph::core::PathPtr warp_path = path->clone();
  graph::core::PathLocalOptimizerPtr path_opt = std::make_shared<graph::core::PathLocalOptimizer>(checker,metrics,logger);
  path_opt->setPath(warp_path);
  path_opt->warp(warp_min_conn_length,warp_min_step_size);
  RCLCPP_INFO_STREAM(node->get_logger(),"Path after warp \n "<<*warp_path);

  display->displayPathAndWaypoints(warp_path,"graph_display",{0.0,1.0,0.0,1});

  RCLCPP_INFO(node->get_logger(),"Simplify on cloned path");
  double simplify_max_conn_length;
  graph::core::get_param(logger,param_ns2,"simplify_max_conn_length",simplify_max_conn_length,0.1);

  graph::core::PathPtr simplify_path = path->clone();
  path_opt->setPath(simplify_path);
  path_opt->simplify(simplify_max_conn_length);
  RCLCPP_INFO_STREAM(node->get_logger(),"Path after simplify \n "<<*simplify_path);

  display->displayPathAndWaypoints(simplify_path,"graph_display",{1.0,1.0,0.0,1});

  return 0;
}

