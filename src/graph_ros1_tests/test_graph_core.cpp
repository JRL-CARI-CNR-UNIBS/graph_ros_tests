#include <ros/package.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <graph_ros1/parallel_moveit_collision_checker.h>
#include <graph_core/informed_sampler.h>
#include <graph_core/solvers/rrt.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/anytime_rrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/metrics.h>
#include <graph_display/graph_display.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_graph_core");
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
    yaml_file_path = "/config/test_graph_core.yaml";
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
  graph_display::DisplayPtr display = std::make_shared<graph_display::Display>(planning_scene,group_name,kinematic_model->getLinkModelNames().back());
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

  // Read start and goal configurations
  std::vector<double> start_vector, goal_vector;
  if (not config["start_configuration"] || not config["goal_configuration"])
  {
    ROS_ERROR_STREAM("Parameter 'start_configuration' and/or 'goal_configuration' not found.");
    return 1;
  }
  else
  {
    start_vector = config["start_configuration"].as<std::vector<double>>();
    goal_vector  = config["goal_configuration" ].as<std::vector<double>>();
  }

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_vector.data(), start_vector.size());
  Eigen::VectorXd goal_conf  = Eigen::Map<Eigen::VectorXd>(goal_vector .data(), goal_vector .size());

  ROS_INFO_STREAM("Start conf: "<<start_conf.transpose());
  ROS_INFO_STREAM("Goal conf: " <<goal_conf .transpose());

  // Set-up planning tools
  int n_threads = 4;
  n_threads = config["parallel_checker_n_threads"].as<int>();

  double checker_resolution = 0.01;
  checker_resolution = config["checker_resolution"].as<double>();

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_graph_core",logger_file);
  graph_core::CollisionCheckerPtr checker = std::make_shared<graph_ros1::ParallelMoveitCollisionChecker>(planning_scene, group_name, logger, n_threads, checker_resolution);
  graph_core::SamplerPtr sampler = std::make_shared<graph_core::InformedSampler>(start_conf,goal_conf,lb,ub,logger);
  graph_core::MetricsPtr metrics = std::make_shared<graph_core::Metrics>(logger);

  std::string planner = "RRT";
  planner = config["planner"].as<std::string>();

  graph_core::TreeSolverPtr solver;
  if(planner == "RRT")
    solver = std::make_shared<graph_core::RRT>(metrics,checker,sampler,logger);
  else if(planner == "RRTCONNECT")
    solver = std::make_shared<graph_core::BiRRT>(metrics,checker,sampler,logger);
  else if(planner == "ANYTIMERRT")
    solver = std::make_shared<graph_core::AnytimeRRT>(metrics,checker,sampler,logger);
  else if(planner == "RRT*")
    solver = std::make_shared<graph_core::RRTStar>(metrics,checker,sampler,logger);
  else
  {
    CNR_ERROR(logger,"Solver not available: "<<planner);
    return 1;
  }

  graph_core::PathPtr solution;
  ros::WallTime tic = ros::WallTime::now();
  if(solver->computePath(start_conf,goal_conf,config,solution,10.0,1000000))
  {
    ROS_INFO_STREAM("Solution found!\n"<<*solution);
    display->displayPathAndWaypoints(solution);
    display->displayTree(solution->getTree(),"graph_display",{0.0,0.0,1.0,0.5});

    std::cout << "Press Enter to rewire the solution with RRT* and informed sampling";
    std::getchar();

    graph_core::RRTStarPtr rrt_star = std::make_shared<graph_core::RRTStar>(metrics,checker,sampler,logger);
    rrt_star->importFromSolver(solver);
    rrt_star->setSolution(solution);
    if(rrt_star->solve(solution,1000000,10.0))
    {
      ROS_INFO_STREAM("Solution found!\n"<<*solution);
      display->clearMarkers();
      display->displayPathAndWaypoints(solution);
      display->displayTree(solution->getTree(),"graph_display",{0.0,0.0,1.0,0.5});
    }
    else
      ROS_ERROR_STREAM("No better solution found");
  }
  else
    ROS_ERROR_STREAM("Solution not found in "<<(ros::WallTime::now()-tic).toSec()<<" seconds");

  return 0;
}

