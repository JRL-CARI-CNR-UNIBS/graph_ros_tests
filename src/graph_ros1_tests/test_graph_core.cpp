#include <ros/package.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <graph_ros1/parallel_moveit_collision_checker.h>

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
  std::string yaml_file_path;
  if(not nh.getParam("yaml_file_path",yaml_file_path))
  {
    yaml_file_path = "/config/test_graph_core.yaml";
    ROS_ERROR_STREAM("yaml_file_path not defined, using "<<yaml_file_path);
  }

  std::string yaml_file = package_path+yaml_file_path;
  ROS_INFO_STREAM("Yaml file: "<<yaml_file);

  std::string txt = "txt";
  cnr_logger::TraceLoggerPtr l = std::make_shared<cnr_logger::TraceLogger>(txt,txt);

//  YAML::Node config;
//  config = YAML::LoadFile(yaml_file);

//  Eigen::VectorXd q;
//  q.setRandom(3,1);
//  graph_core::Node n(q);

//  ROS_INFO_STREAM("q"<<n.getReservedFlagsNumber());



//  try {
//    config = YAML::LoadFile(yaml_file_path);
//  } catch (const YAML::Exception& e) {
//    ROS_ERROR_STREAM("Error loading YAML file");
//    return 1;
//  }

  // Get the robot description
//  std::string group_name;
//  if (config["group_name"])
//    group_name = config["group_name"].as<std::string>();
//  else
//  {
//    ROS_ERROR_STREAM("Parameter 'group_name' not found.");
//    return 1;
//  }

//  moveit::planning_interface::MoveGroupInterface move_group(group_name);
//  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
//  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
//  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

//  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
//  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

//  unsigned int dof = joint_names.size();
//  Eigen::VectorXd lb(dof);
//  Eigen::VectorXd ub(dof);

//  for (unsigned int idx = 0; idx < dof; idx++)
//  {
//    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
//    if (bounds.position_bounded_)
//    {
//      lb(idx) = bounds.min_position_;
//      ub(idx) = bounds.max_position_;
//    }
//  }

//  // Update the planning scene
//  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
//  moveit_msgs::GetPlanningScene ps_srv;
//  if (!ps_client.waitForExistence(ros::Duration(10)))
//  {
//    ROS_ERROR("Unable to connect to /get_planning_scene");
//    return 1;
//  }

//  if (!ps_client.call(ps_srv))
//  {
//    ROS_ERROR("Call to srv not ok");
//    return 1;
//  }

//  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
//  {
//    ROS_ERROR("unable to update planning scene");
//    return 1;
//  }

  // Read start and goal configurations

//  std::vector<double> start_vector, goal_vector;
//  if (not config["start_configuration"] || not config["goal_configuration"])
//  {
//    ROS_ERROR_STREAM("Parameter 'start_configuration' and/or 'goal_configuration' not found.");
//    return 1;
//  }
//  else
//  {
//    start_vector = config["start_configuration"].as<std::vector<double>>();
//    goal_vector  = config["goal_configuration" ].as<std::vector<double>>();
//  }

//  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start_vector.data(), start_vector.size());
//  Eigen::VectorXd goal_conf  = Eigen::Map<Eigen::VectorXd>(goal_vector .data(), goal_vector .size());

//  ROS_INFO_STREAM("Start conf: "<<start_conf.transpose());
//  ROS_INFO_STREAM("Goal conf: " <<goal_conf .transpose());

  return 0;
}

