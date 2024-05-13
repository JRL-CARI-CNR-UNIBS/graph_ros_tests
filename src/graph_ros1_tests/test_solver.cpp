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
  ros::NodeHandle nh;

  // Load logger configuration file
  std::string package_name = "graph_ros1_tests";
  std::string package_path = ros::package::getPath(package_name);

  if (package_path.empty())
  {
    ROS_ERROR_STREAM("Failed to get path for package '" << package_name);
    return 1;
  }

  std::string logger_file = package_path+"/config/logger_param.yaml";
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_solver",logger_file);

  // Get the robot description
  std::string param_ns1 = "/"+package_name;
  std::string param_ns2 = param_ns1+"/test_solver";
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

  // Read start and goal configurations
  Eigen::VectorXd start_conf, goal_conf;
  if(not graph::core::get_param(logger,param_ns2,"start_configuration",start_conf))
    return 1;
  if(not graph::core::get_param(logger,param_ns2,"goal_configuration",goal_conf))
    return 1;

  ROS_INFO_STREAM("Start conf: "<<start_conf.transpose());
  ROS_INFO_STREAM("Goal conf: " <<goal_conf .transpose());

  ROS_INFO_STREAM("LB conf: " <<lb .transpose());
  ROS_INFO_STREAM("UB conf: " <<ub .transpose());

  // Set-up planning tools
  graph::core::GoalCostFunctionPtr goal_cost_fcn = std::make_shared<graph::core::GoalCostFunction>();

  // Load collision checker plugin
  pluginlib::ClassLoader<graph::ros1::CollisionCheckerBasePlugin> checker_plugin_loader("graph_ros1", "graph::ros1::CollisionCheckerBasePlugin");

  std::string checker_plugin_name;
  graph::core::get_param(logger,param_ns2,"checker_plugin",checker_plugin_name,(std::string)"graph::ros1::ParallelMoveitCollisionCheckerPlugin");

  ROS_INFO_STREAM("Loading checker "<<checker_plugin_name);
  boost::shared_ptr<graph::ros1::CollisionCheckerBasePlugin> checker_plugin = checker_plugin_loader.createInstance(checker_plugin_name);

  ROS_INFO_STREAM("Configuring checker plugin ");
  checker_plugin->init(nh,param_ns2,planning_scene,logger);
  graph::core::CollisionCheckerPtr checker = checker_plugin->getCollisionChecker();

  // Load collision sampler plugin
  pluginlib::ClassLoader<graph::ros1::SamplerBasePlugin> sampler_plugin_loader("graph_ros1", "graph::ros1::SamplerBasePlugin");

  std::string sampler_plugin_name;
  graph::core::get_param(logger,param_ns2,"sampler_plugin",sampler_plugin_name,(std::string)"graph::ros1::InformedSamplerPlugin");

  ROS_INFO_STREAM("Loading sampler "<<sampler_plugin_name);
  boost::shared_ptr<graph::ros1::SamplerBasePlugin> sampler_plugin = sampler_plugin_loader.createInstance(sampler_plugin_name);

  ROS_INFO_STREAM("Configuring sampler plugin ");
  Eigen::VectorXd scale(dof); scale.setOnes(dof,1);

  sampler_plugin->init(nh,param_ns2,start_conf,goal_conf,lb,ub,scale,logger);
  graph::core::SamplerPtr sampler = sampler_plugin->getSampler();

  // Load collision metrics plugin
  pluginlib::ClassLoader<graph::ros1::MetricsBasePlugin> metrics_plugin_loader("graph_ros1", "graph::ros1::MetricsBasePlugin");

  std::string metrics_plugin_name = "graph::ros1::EuclideanMetricsPlugin";
  graph::core::get_param(logger,param_ns2,"metrics_plugin",metrics_plugin_name,(std::string)"graph::ros1::EuclideanMetricsPlugin");

  ROS_INFO_STREAM("Loading metrics "<<metrics_plugin_name);
  boost::shared_ptr<graph::ros1::MetricsBasePlugin> metrics_plugin = metrics_plugin_loader.createInstance(metrics_plugin_name);

  ROS_INFO_STREAM("Configuring metrics plugin ");
  metrics_plugin->init(nh,param_ns2,logger);
  graph::core::MetricsPtr metrics = metrics_plugin->getMetrics();

  // Load planner plugin
  pluginlib::ClassLoader<graph::ros1::TreeSolverPlugin> solver_plugin_loader("graph_ros1", "graph::ros1::TreeSolverPlugin");

  std::string planner_plugin_name = "graph::ros1::RRTPlugin";
  graph::core::get_param(logger,param_ns2,"planner_plugin",planner_plugin_name,(std::string)"graph::ros1::RRTPlugin");

  ROS_INFO_STREAM("Loading plugin "<<planner_plugin_name);
  boost::shared_ptr<graph::ros1::TreeSolverPlugin> solver_plugin = solver_plugin_loader.createInstance(planner_plugin_name);

  ROS_INFO_STREAM("Configuring solver plugin ");
  solver_plugin->init(nh,param_ns2,metrics,checker,sampler,goal_cost_fcn,logger);
  graph::core::TreeSolverPtr solver = solver_plugin->getSolver();

  graph::core::PathPtr solution;
  ros::WallTime tic = ros::WallTime::now();
  graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_conf,logger);
  graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal_conf,logger);

  if(solver->computePath(start_node,goal_node,param_ns2,solution,10.0,1000000))
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

    YAML::Node yaml_path, yaml_path_ns, yaml_tree, yaml_tree_ns;

    yaml_path["path"] = solution->toYAML();
    yaml_tree["tree"] = solution->getTree()->toYAML();

    yaml_path_ns[package_name] = yaml_path;
    yaml_tree_ns[package_name] = yaml_tree;

    std::ofstream fout_path(package_path+"/config/path.yaml");
    std::ofstream fout_tree(package_path+"/config/tree.yaml");

    if (fout_path.is_open())
    {
      fout_path << yaml_path_ns;
      fout_path.close();
    }
    else
      CNR_ERROR(logger,"Error opening 'path.yaml' for writing.");

    if (fout_tree.is_open())
    {
      fout_tree << yaml_tree_ns;
      fout_tree.close();
    }
    else
      CNR_ERROR(logger,"Error opening 'tree.yaml' for writing.");

  }
  else
    ROS_ERROR_STREAM("Solution not found in "<<(ros::WallTime::now()-tic).toSec()<<" seconds");

  return 0;
}

