// ROS and Moveit related libraries
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Graph core libraries
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/plugins/solvers/tree_solver_plugin.h>
#include <graph_core/plugins/samplers/sampler_base_plugin.h>
#include <graph_core/plugins/metrics/metrics_base_plugin.h>

#include <moveit_collision_checker/plugins/collision_checkers/moveit_collision_checker_base_plugin.h>
#include <graph_display/graph_display.h>

// Class loader
#include <cnr_class_loader/multi_library_class_loader.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto node = rclcpp::Node::make_shared("test_solver", options);

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
  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("test_solver",logger_file);

  // Get the robot description
  std::string param_ns1 = "/"+package_name;
  std::string param_ns2 = param_ns1+"/test_solver";
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

  // Read start and goal configurations
  Eigen::VectorXd start_conf, goal_conf;
  if(not graph::core::get_param(logger,param_ns2,"start_configuration",start_conf))
    return 1;
  if(not graph::core::get_param(logger,param_ns2,"goal_configuration",goal_conf))
    return 1;

  RCLCPP_INFO_STREAM(node->get_logger(),"Start conf: "<<start_conf.transpose());
  RCLCPP_INFO_STREAM(node->get_logger(),"Goal conf: " <<goal_conf .transpose());

  RCLCPP_INFO_STREAM(node->get_logger(),"LB conf: " <<lb .transpose());
  RCLCPP_INFO_STREAM(node->get_logger(),"UB conf: " <<ub .transpose());

  // Set-up planning tools
  graph::core::GoalCostFunctionPtr goal_cost_fcn = std::make_shared<graph::core::GoalCostFunction>();

  // Set-up the class laoder
  cnr_class_loader::MultiLibraryClassLoader loader(false);
  std::vector<std::string> libraries;
  if(not graph::core::get_param(logger,param_ns2,"libraries",libraries))
    return 1;

  for(const std::string& lib:libraries)
    loader.loadLibrary(lib);

  // Load collision checker plugin
  std::string checker_plugin_name;
  graph::core::get_param(logger,param_ns2,"checker_plugin",checker_plugin_name,(std::string)"graph::ros1::ParallelMoveitCollisionCheckerPlugin");

  RCLCPP_INFO_STREAM(node->get_logger(),"Loading checker "<<checker_plugin_name);
  std::shared_ptr<graph::ros2::MoveitCollisionCheckerBasePlugin> checker_plugin = loader.createInstance<graph::ros2::MoveitCollisionCheckerBasePlugin>(checker_plugin_name);

  RCLCPP_INFO(node->get_logger(),"Configuring checker plugin ");
  checker_plugin->init(param_ns2,planning_scene,logger);
  graph::core::CollisionCheckerPtr checker = checker_plugin->getCollisionChecker();

  // Load sampler plugin
  std::string sampler_plugin_name;
  graph::core::get_param(logger,param_ns2,"sampler_plugin",sampler_plugin_name,(std::string)"graph::core::InformedSamplerPlugin");

  RCLCPP_INFO_STREAM(node->get_logger(),"Loading sampler "<<sampler_plugin_name);
  std::shared_ptr<graph::core::SamplerBasePlugin> sampler_plugin = loader.createInstance<graph::core::SamplerBasePlugin>(sampler_plugin_name);

  RCLCPP_INFO(node->get_logger(),"Configuring sampler plugin ");
  Eigen::VectorXd scale(dof); scale.setOnes(dof,1);

  sampler_plugin->init(param_ns2,start_conf,goal_conf,lb,ub,scale,logger);
  graph::core::SamplerPtr sampler = sampler_plugin->getSampler();

  // Load metrics plugin
  std::string metrics_plugin_name;
  graph::core::get_param(logger,param_ns2,"metrics_plugin",metrics_plugin_name,(std::string)"graph::core::EuclideanMetricsPlugin");

  RCLCPP_INFO_STREAM(node->get_logger(),"Loading metrics "<<metrics_plugin_name);
  std::shared_ptr<graph::core::MetricsBasePlugin> metrics_plugin = loader.createInstance<graph::core::MetricsBasePlugin>(metrics_plugin_name);

  RCLCPP_INFO(node->get_logger(),"Configuring metrics plugin ");
  metrics_plugin->init(param_ns2,logger);
  graph::core::MetricsPtr metrics = metrics_plugin->getMetrics();

  // Load planner plugin
  std::string planner_plugin_name;
  graph::core::get_param(logger,param_ns2,"planner_plugin",planner_plugin_name,(std::string)"graph::core::RRTPlugin");

  RCLCPP_INFO_STREAM(node->get_logger(),"Loading plugin "<<planner_plugin_name);
  std::shared_ptr<graph::core::TreeSolverPlugin> solver_plugin = loader.createInstance<graph::core::TreeSolverPlugin>(planner_plugin_name);

  RCLCPP_INFO(node->get_logger(),"Configuring solver plugin ");
  solver_plugin->init(param_ns2,metrics,checker,sampler,goal_cost_fcn,logger);
  graph::core::TreeSolverPtr solver = solver_plugin->getSolver();

  graph::core::PathPtr solution;
  auto tic = std::chrono::system_clock::now();
  graph::core::NodePtr start_node = std::make_shared<graph::core::Node>(start_conf,logger);
  graph::core::NodePtr goal_node = std::make_shared<graph::core::Node>(goal_conf,logger);

  if(solver->computePath(start_node,goal_node,param_ns2,solution,10.0,1000000))
  {
    RCLCPP_INFO_STREAM(node->get_logger(),"Solution found!\n"<<*solution);
    display->displayPathAndWaypoints(solution);
    display->displayTree(solution->getTree(),"graph_display",{0.0,0.0,1.0,0.15});

    std::cout << "Press Enter to rewire the solution with RRT* and informed sampling";
    std::getchar();

    graph::core::RRTStarPtr rrt_star = std::make_shared<graph::core::RRTStar>(metrics,checker,sampler,logger);
    rrt_star->importFromSolver(solver);
    rrt_star->setSolution(solution);
    if(rrt_star->solve(solution,1000000,10.0))
    {
      RCLCPP_INFO_STREAM(node->get_logger(),"Solution found!\n"<<*solution);
      display->clearMarkers();
      display->displayPathAndWaypoints(solution);
      display->displayTree(solution->getTree(),"graph_display",{0.0,0.0,1.0,0.15});
    }
    else
      RCLCPP_ERROR(node->get_logger(),"No better solution found");

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
      RCLCPP_ERROR(node->get_logger(),"Error opening 'path.yaml' for writing.");

    if (fout_tree.is_open())
    {
      fout_tree << yaml_tree_ns;
      fout_tree.close();
    }
    else
      RCLCPP_ERROR(node->get_logger(),"Error opening 'tree.yaml' for writing.");

  }
  else
    RCLCPP_ERROR_STREAM(node->get_logger(),"Solution not found in "<<std::chrono::duration<double>((std::chrono::system_clock::now()-tic)).count()<<" seconds");

  return 0;
}

