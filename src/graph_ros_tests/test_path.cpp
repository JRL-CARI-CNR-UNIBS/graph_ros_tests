// ROS and Moveit related libraries
#include <ros/package.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Graph core libraries
#include <graph_display/graph_display.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <moveit_collision_checker/collision_checkers/parallel_moveit_collision_checker.h>
#include <graph_core/solvers/path_optimizers/path_local_optimizer.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_path");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::NodeHandle nh;

  ros::WallDuration(5).sleep();

  // Load logger configuration file
  std::string package_name = "graph_ros_tests";
  std::string package_path = ros::package::getPath(package_name);

  if (package_path.empty())
  {
    ROS_ERROR_STREAM("Failed to get path for package '" << package_name);
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
  int n_threads;
  graph::core::get_param(logger,param_ns2,"parallel_checker_n_threads",n_threads,4);

  double checker_resolution;
  graph::core::get_param(logger,param_ns2,"checker_resolution",checker_resolution,0.01);

  double max_distance;
  graph::core::get_param(logger,param_ns2,"max_distance",max_distance,1.0);

  bool use_kdtree = true;
  graph::core::get_param(logger,param_ns2,"use_kdtree",use_kdtree,true);

  graph::core::CollisionCheckerPtr checker = std::make_shared<graph::collision_check::ParallelMoveitCollisionChecker>(planning_scene, group_name, logger, n_threads, checker_resolution);
  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  graph::core::PathPtr path;
  graph::core::get_param(logger,param_ns1,"path",path,metrics,checker);

  graph::core::TreePtr tree;
  graph::core::get_param(logger,param_ns1,"tree",tree,metrics,checker);

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
  double warp_min_conn_length;
  graph::core::get_param(logger,param_ns2,"warp_min_conn_length",warp_min_conn_length,0.1);

  double warp_min_step_size;
  graph::core::get_param(logger,param_ns2,"warp_min_step_size",warp_min_step_size,0.1);

  graph::core::PathPtr warp_path = path->clone();
  graph::core::PathLocalOptimizerPtr path_opt = std::make_shared<graph::core::PathLocalOptimizer>(checker,metrics,logger);
  path_opt->setPath(warp_path);
  path_opt->warp(warp_min_conn_length,warp_min_step_size);
  ROS_INFO_STREAM("Path after warp \n "<<*warp_path);

  display->displayPathAndWaypoints(warp_path,"graph_display",{0.0,1.0,0.0,1});

  ROS_INFO("Simplify on cloned path");
  double simplify_max_conn_length;
  graph::core::get_param(logger,param_ns2,"simplify_max_conn_length",simplify_max_conn_length,0.1);

  graph::core::PathPtr simplify_path = path->clone();
  path_opt->setPath(simplify_path);
  path_opt->simplify(simplify_max_conn_length);
  ROS_INFO_STREAM("Path after simplify \n "<<*simplify_path);

  display->displayPathAndWaypoints(simplify_path,"graph_display",{1.0,1.0,0.0,1});

  return 0;
}

