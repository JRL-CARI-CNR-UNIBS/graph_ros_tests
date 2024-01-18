#include <ros/package.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <graph_display/graph_display.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_ros1/collision_checkers/parallel_moveit_collision_checker.h>
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_path");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  // Load YAML file into a YAML::Node
  std::string package_name = "graph_ros1_tests";
  std::string package_path = ros::package::getPath(package_name);

  if (package_path.empty())
  {
    ROS_ERROR_STREAM("Failed to get path for package '" << package_name);
    return 1;
  }

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  std::string yaml_file_path;
  if(not pnh.getParam("yaml_file_path",yaml_file_path))
  {
    yaml_file_path = "/config/test_path.yaml";
    ROS_ERROR_STREAM("yaml_file_path not defined, using "<<yaml_file_path);
  }

  std::string yaml_file = package_path+yaml_file_path;
  ROS_INFO_STREAM("Yaml file: "<<yaml_file);

  YAML::Node config;

  try {
    config = YAML::LoadFile(yaml_file);
  } catch (const YAML::Exception& e) {
    ROS_ERROR_STREAM("Error loading YAML file: "<<e.what());
    return 1;
  }

  // Get the robot description
  std::string group_name;
  if (config["group_name"])
    group_name = config["group_name"].as<std::string>();
  else
  {
    ROS_ERROR_STREAM("Parameter 'group_name' not found.");
    return 1;
  }

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);
  const robot_state::JointModelGroup* joint_model_group =  kinematic_model->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  // Update the planning scene
  graph::display::DisplayPtr display = std::make_shared<graph::display::Display>(planning_scene,group_name,kinematic_model->getLinkModelNames().back());
  kinematic_model->getLinkModelNames();
  ros::WallDuration(1).sleep();

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
  moveit_msgs::GetPlanningScene ps_srv;
  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("Unable to connect to /get_planning_scene");
    return 1;
  }

  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("Call to srv not ok");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 1;
  }

  // Set-up planning tools
  int n_threads = 4;
  n_threads = config["parallel_checker_n_threads"].as<int>();

  double checker_resolution = 0.01;
  checker_resolution = config["checker_resolution"].as<double>();

  double max_distance = 1.0;
  max_distance = config["max_distance"].as<double>();

  bool use_kdtree = true;
  use_kdtree = config["use_kdtree"].as<bool>();

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_path",logger_file);
  graph::core::CollisionCheckerPtr checker = std::make_shared<graph::ros1::ParallelMoveitCollisionChecker>(planning_scene, group_name, logger, n_threads, checker_resolution);
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  std::string path_file_name = "config/path.yaml";
  path_file_name = config["path_file_name"].as<std::string>();

  std::string tree_file_name = "config/tree.yaml";
  tree_file_name = config["tree_file_name"].as<std::string>();

  YAML::Node yaml_path, yaml_tree;

  std::string tree_file = package_path+tree_file_name;
  try {
    yaml_tree = YAML::LoadFile(tree_file);
  } catch (const YAML::Exception& e) {
    ROS_ERROR_STREAM("Error loading YAML file: "<<e.what());
    ROS_ERROR_STREAM("Tree file name: "<<tree_file);
    return 1;
  }

  std::string path_file = package_path+path_file_name;
  try {
    yaml_path = YAML::LoadFile(path_file);
  } catch (const YAML::Exception& e) {
    ROS_ERROR_STREAM("Error loading YAML file: "<<e.what());
    ROS_ERROR_STREAM("Path file name: "<<path_file);
    return 1;
  }

  graph::core::PathPtr path = graph::core::Path::fromYAML(yaml_path,metrics,checker,logger);
  graph::core::TreePtr tree = graph::core::Tree::fromYAML(yaml_tree,max_distance,checker,metrics,logger,use_kdtree);
  path->setTree(tree);

  tree->print_full_tree_ = true;

  ROS_INFO_STREAM("Tree:\n"<<*tree);
  ROS_INFO_STREAM("Path:\n"<<*path);

  ros::WallDuration(1.0).sleep();
  display->clearMarkers();
  display->displayPathAndWaypoints(path);
  display->displayTree(tree,"graph_display",{0.0,0.0,1.0,0.01});

  ROS_INFO("Flip the path");
  path->flip();
  ROS_INFO_STREAM("Flipped path\n "<<*path);

  ROS_INFO("Warp on cloned path");
  double warp_min_conn_length = 0.1;
  warp_min_conn_length = config["warp_min_conn_length"].as<double>();

  double warp_min_step_size = 0.1;
  warp_min_step_size = config["warp_min_step_size"].as<double>();

  graph::core::PathPtr warp_path = path->clone();
  graph::core::PathLocalOptimizerPtr path_opt = std::make_shared<graph::core::PathLocalOptimizer>(checker,metrics,logger);
  path_opt->setPath(warp_path);
  path_opt->warp(warp_min_conn_length,warp_min_step_size);
  ROS_INFO_STREAM("Path after warp \n "<<*warp_path);

  display->displayPathAndWaypoints(warp_path,"graph_display",{0.0,1.0,0.0,1});

  ROS_INFO("Simplify on cloned path");
  double simplify_max_conn_length = 0.1;
  simplify_max_conn_length = config["simplify_max_conn_length"].as<double>();

  graph::core::PathPtr simplify_path = path->clone();
  path_opt->setPath(simplify_path);
  path_opt->simplify(simplify_max_conn_length);
  ROS_INFO_STREAM("Path after simplify \n "<<*simplify_path);

  display->displayPathAndWaypoints(simplify_path,"graph_display",{1.0,1.0,0.0,1});

  return 0;
}

