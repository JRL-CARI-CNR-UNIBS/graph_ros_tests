<?xml version="1.0"?>
<launch>

<!--  Load parameters in cnr_param server-->
<node pkg="graph_ros1_tests"
type="load_params.sh" name="load_params"
args="test_path.yaml path.yaml tree.yaml"
output="screen">
</node>

<!--  Launche the cell and load the obstacles-->
<include file="$(find cartesian_moveit_config)/launch/demo.launch"> </include>

<rosparam command="load" file="$(find graph_ros1_tests)/config/scene.yaml"/>
<node pkg="object_loader" name="context_manager_node" type="context_manager_node" respawn="true" output="screen"/>
<node pkg="object_loader" name="scene_spawner" type="scene_spawner" output="screen"/>

<!--  Launche the test node-->
<node pkg="graph_ros1_tests"
name="test_path"
type="test_path"
output="screen">
</node>

</launch>
<!--          launch-prefix="gdb -ex run -X-args" >-->


# This Python file uses the following encoding: utf-8
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
  config_folder = PathJoinSubstitution([FindPackageShare("graph_ros_tests"),"config"])

  return LaunchDescription([
    ExecuteProcess(
        cmd = [
          FindExecutable(name="cnr_param_server"),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "test_path.yaml"        # load the behavior tree plugins and the tree
          ]),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "path.yaml"    # load the parameters of each skill plugin
          ]),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "tree.yaml"             # list of precomputed trajectories
          ]),
        ],
        shell=False
      ),

    Node(
      package="graph_ros_tests",
      executable="test_path",
      output="screen",
      namespace="test_path",
      ros_arguments=["--log-level", "info"]
      )
])
