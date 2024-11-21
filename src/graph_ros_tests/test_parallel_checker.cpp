// ROS and Moveit related libraries
#include <ros/package.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Graph core libraries
#include <graph_display/graph_display.h>
#include <graph_core/samplers/uniform_sampler.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <moveit_collision_checker/collision_checkers/parallel_moveit_collision_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_parallel_checker");
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
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_parallel_checker",logger_file);

  // Get the robot description
  std::string param_ns1 = "/"+package_name;
  std::string param_ns2 = param_ns1+"/test_parallel_checker";
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

  int n_iter;
  graph::core::get_param(logger,param_ns2,"n_iter",n_iter,100000);

  graph::collision_check::MoveitCollisionCheckerPtr checker = std::make_shared<graph::collision_check::MoveitCollisionChecker>(planning_scene, group_name, logger, checker_resolution);
  graph::collision_check::ParallelMoveitCollisionCheckerPtr parallel_checker = std::make_shared<graph::collision_check::ParallelMoveitCollisionChecker>(planning_scene, group_name, logger, n_threads, checker_resolution);

  graph::core::MetricsPtr metrics = std::make_shared<graph::core::EuclideanMetrics>(logger);

  graph::core::PathPtr path;
  graph::core::get_param(logger,param_ns1,"path",path,metrics,checker);

  graph::core::graph_time_point tic;
  double parallel_mean = 0.0;
  double base_mean = 0.0;
  CNR_INFO(logger,cnr_logger::BW()<<"======> CHECKING THE PATH <======"<<cnr_logger::RESET());
  for(unsigned int i=0;i<100;i++)
  {
    path->setChecker(checker);
    tic = graph::core::graph_time::now();
    path->isValid(checker);
    double time_base_checker = graph::core::toSeconds(graph::core::graph_time::now(),tic)*1000.0;

    base_mean += time_base_checker;

    path->setChecker(parallel_checker);
    tic = graph::core::graph_time::now();
    path->isValid(parallel_checker);
    double time_parallel_checker = graph::core::toSeconds(graph::core::graph_time::now(),tic)*1000.0;

    parallel_mean += time_parallel_checker;

    CNR_INFO(logger,"parallel checker time: "<<time_parallel_checker<<"ms --- base checker time: "<<time_base_checker<<"ms");
  }

  CNR_WARN(logger,"PATH: mean time parallel checker: "<<parallel_mean/100.0<<"ms mean time base checker: "<<base_mean/100.0<<"ms");
  CNR_WARN(logger,"PATH: parallel checker is "<<(base_mean/100.0)/(parallel_mean/100.0)<<" times faster");

  CNR_INFO(logger,cnr_logger::BW()<<"======> CHECKING RANDOM CONNECTIONS <======"<<cnr_logger::RESET());
  graph::core::UniformSampler sampler = graph::core::UniformSampler(lb,ub,logger);

  parallel_mean = 0.0;
  base_mean = 0.0;
  for(int i=0;i<n_iter;i++)
  {
    auto q1 = sampler.sample();
    auto q2 = sampler.sample();

    tic = graph::core::graph_time::now();
    bool res1 = parallel_checker->checkConnection(q1,q2);
    double time_parallel_checker = graph::core::toSeconds(graph::core::graph_time::now(),tic)*1000.0;

    parallel_mean += time_parallel_checker;

    tic = graph::core::graph_time::now();
    bool res2 = checker->checkConnection(q1,q2);
    double time_base_checker = graph::core::toSeconds(graph::core::graph_time::now(),tic)*1000.0;

    base_mean += time_base_checker;

    CNR_INFO(logger,"idx: "<<i<<" -> dist: "<<(q2-q1).norm()<<" -- parallel checker time: "<<time_parallel_checker<<"ms -- base checker time: "<<time_base_checker<<"ms");

    if(res1 != res2)
      throw std::runtime_error("res1 != res2");
  }

  CNR_WARN(logger,"CONNECTIONS: mean time parallel checker: "<<parallel_mean/double(n_iter)<<"ms mean time base checker: "<<base_mean/double(n_iter)<<"ms");
  CNR_WARN(logger,"CONNECTIONS: parallel checker is "<<(base_mean/double(n_iter))/(parallel_mean/double(n_iter))<<" times faster");

  return 0;
}

