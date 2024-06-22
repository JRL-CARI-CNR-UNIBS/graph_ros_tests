// ROS and Moveit related libraries
#include <ros/package.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Graph core libraries
#include <graph_core/plugins/samplers/sampler_base_plugin.h>
#include <graph_display/graph_display.h>

// Class loader
#include <cnr_class_loader/multi_library_class_loader.hpp>
#include <thread>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_solver");
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

  // Set-up the class laoder
  cnr_class_loader::MultiLibraryClassLoader loader(false);
  std::vector<std::string> libraries;
  if(not graph::core::get_param(logger,param_ns2,"libraries",libraries))
    return 1;

  for(const std::string& lib:libraries)
    loader.loadLibrary(lib);

  // Load sampler plugin
  std::string sampler_plugin_name;
  graph::core::get_param(logger,param_ns2,"sampler_plugin",sampler_plugin_name,(std::string)"graph::core::BallSamplerPlugin");

  ROS_INFO_STREAM("Loading sampler "<<sampler_plugin_name);
  std::shared_ptr<graph::core::SamplerBasePlugin> sampler_plugin = loader.createInstance<graph::core::SamplerBasePlugin>(sampler_plugin_name);

  ROS_INFO_STREAM("Configuring sampler plugin ");
  Eigen::VectorXd scale(dof); scale.setOnes(dof,1);

  sampler_plugin->init(param_ns2,start_conf,goal_conf,lb,ub,scale,logger);
  graph::core::SamplerPtr sampler = sampler_plugin->getSampler();
  double cost = std::numeric_limits<double>::infinity();
  sampler->setCost(cost);
  display->clearMarkers();
  display->clearMarkers();
  display->changeNodeSize({0.01,0.01,0.01});
  ROS_INFO_STREAM("Pause ");

  ros::Duration(3).sleep();

  for(unsigned int i=0;i<1000000;i++)
  {
    Eigen::VectorXd q = sampler->sample();
    graph::core::NodePtr n = std::make_shared<graph::core::Node>(q,logger);
    display->displayNode(n,"pathplan",{0,0,1,1});

    ROS_INFO_STREAM("Sample "<<q.transpose());

    for(size_t j=0;j<3;j++)
    {
      if((q[j]<(start_conf[j]-cost)) ||(q[j]>(start_conf[j]+cost)))
          ROS_ERROR("Q OUT OF BOUNDS");
    }
    ros::WallDuration(0.0001).sleep();
  }

  return 0;
}

