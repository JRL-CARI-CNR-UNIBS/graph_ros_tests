#include <ros/package.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <graph_core/solvers/rrt_star.h>
#include <graph_display/graph_display.h>

#include <pluginlib/class_loader.h>
#include <graph_ros1/plugins/solvers/tree_solver_plugin.h>
#include <graph_ros1/plugins/collision_checkers/collision_checker_base_plugin.h>
#include <graph_ros1/plugins/samplers/sampler_base_plugin.h>
#include <graph_ros1/plugins/metrics/metrics_base_plugin.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_solver");
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
    yaml_file_path = "/config/test_solver.yaml";
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
  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_solver",logger_file);
  graph::core::GoalCostFunctionPtr goal_cost_fcn = std::make_shared<graph::core::GoalCostFunction>();

  // Load collision checker plugin
  pluginlib::ClassLoader<graph::ros1::CollisionCheckerBasePlugin> checker_plugin_loader("graph_ros1", "graph::ros1::CollisionCheckerBasePlugin");

  std::string checker_plugin_name = "graph::ros1::ParallelMoveitCollisionCheckerPlugin";
  checker_plugin_name = config["checker_plugin"].as<std::string>();

  ROS_INFO_STREAM("Loading checker "<<checker_plugin_name);
  boost::shared_ptr<graph::ros1::CollisionCheckerBasePlugin> checker_plugin = checker_plugin_loader.createInstance(checker_plugin_name);

  ROS_INFO_STREAM("Configuring checker plugin ");
  checker_plugin->init(nh,planning_scene,logger);
  graph::core::CollisionCheckerPtr checker = checker_plugin->getCollisionChecker();

  // Load collision sampler plugin
  pluginlib::ClassLoader<graph::ros1::SamplerBasePlugin> sampler_plugin_loader("graph_ros1", "graph::ros1::SamplerBasePlugin");

  std::string sampler_plugin_name = "graph::ros1::InformedSamplerPlugin";
  sampler_plugin_name = config["sampler_plugin"].as<std::string>();

  ROS_INFO_STREAM("Loading sampler "<<sampler_plugin_name);
  boost::shared_ptr<graph::ros1::SamplerBasePlugin> sampler_plugin = sampler_plugin_loader.createInstance(sampler_plugin_name);

  ROS_INFO_STREAM("Configuring sampler plugin ");
  Eigen::VectorXd scale(dof); scale.setOnes(dof,1);
  sampler_plugin->init(nh,start_conf,goal_conf,lb,ub,scale,logger);
  graph::core::SamplerPtr sampler = sampler_plugin->getSampler();

  // Load collision metrics plugin
  pluginlib::ClassLoader<graph::ros1::MetricsBasePlugin> metrics_plugin_loader("graph_ros1", "graph::ros1::MetricsBasePlugin");

  std::string metrics_plugin_name = "graph::ros1::EuclideanMetricsPlugin";
  metrics_plugin_name = config["metrics_plugin"].as<std::string>();

  ROS_INFO_STREAM("Loading metrics "<<metrics_plugin_name);
  boost::shared_ptr<graph::ros1::MetricsBasePlugin> metrics_plugin = metrics_plugin_loader.createInstance(metrics_plugin_name);

  ROS_INFO_STREAM("Configuring metrics plugin ");
  metrics_plugin->init(nh,logger);
  graph::core::MetricsPtr metrics = metrics_plugin->getMetrics();

  // Load planner plugin
  pluginlib::ClassLoader<graph::ros1::TreeSolverPlugin> solver_plugin_loader("graph_ros1", "graph::ros1::TreeSolverPlugin");

  std::string planner_plugin_name = "graph::ros1::RRTPlugin";
  planner_plugin_name = config["planner_plugin"].as<std::string>();

  ROS_INFO_STREAM("Loading plugin "<<planner_plugin_name);
  boost::shared_ptr<graph::ros1::TreeSolverPlugin> solver_plugin = solver_plugin_loader.createInstance(planner_plugin_name);

  ROS_INFO_STREAM("Configuring solver plugin ");
  solver_plugin->init(nh,metrics,checker,sampler,goal_cost_fcn,logger);
  graph::core::TreeSolverPtr solver = solver_plugin->getSolver();

  graph::core::PathPtr solution;
  ros::WallTime tic = ros::WallTime::now();
  graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_conf,logger);
  graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal_conf,logger);

  if(solver->computePath(start_node,goal_node,config,solution,10.0,1000000))
  {
    ROS_INFO_STREAM("Solution found!\n"<<*solution);
    display->displayPathAndWaypoints(solution);
    display->displayTree(solution->getTree(),"graph_display",{0.0,0.0,1.0,0.15});

    std::cout << "Press Enter to rewire the solution with RRT* and informed sampling";
    std::getchar();

    graph::core::RRTStarPtr rrt_star = std::make_shared<graph::core::RRTStar>(metrics,checker,sampler,logger);
    rrt_star->importFromSolver(solver);
    rrt_star->setSolution(solution);
    if(rrt_star->solve(solution,1000000,10.0))
    {
      ROS_INFO_STREAM("Solution found!\n"<<*solution);
      display->clearMarkers();
      display->displayPathAndWaypoints(solution);
      display->displayTree(solution->getTree(),"graph_display",{0.0,0.0,1.0,0.15});
    }
    else
      ROS_ERROR_STREAM("No better solution found");

    YAML::Node yaml_path = solution->toYAML();
    YAML::Node yaml_tree = solution->getTree()->toYAML();

    std::ofstream fout_path(package_path+"/config/path.yaml");
    std::ofstream fout_tree(package_path+"/config/tree.yaml");

    if (fout_path.is_open())
    {
      fout_path << yaml_path;
      fout_path.close();
    }
    else
      CNR_ERROR(logger,"Error opening 'path.yaml' for writing.");

    if (fout_tree.is_open())
    {
      fout_tree << yaml_tree;
      fout_tree.close();
    }
    else
      CNR_ERROR(logger,"Error opening 'tree.yaml' for writing.");

  }
  else
    ROS_ERROR_STREAM("Solution not found in "<<(ros::WallTime::now()-tic).toSec()<<" seconds");

  return 0;
}

