<?xml version="1.0"?>
<launch>

<!--  Load parameters in cnr_param server-->
<node pkg="graph_ros1_tests"
type="load_params.sh" name="load_params"
args="test_solver.yaml"
output="screen">
</node>

<!--  Launche the cell and load the obstacles-->
<include file="$(find cartesian_moveit_config)/launch/demo.launch"> </include>

<rosparam command="load" file="$(find graph_ros1_tests)/config/scene.yaml"/>
<node pkg="object_loader" name="context_manager_node" type="context_manager_node" respawn="true" output="screen"/>
<node pkg="object_loader" name="scene_spawner" type="scene_spawner" output="screen"/>

<!--  Launche the test node-->
<node pkg="graph_ros1_tests"
name="test_solver"
type="test_solver"
output="screen"
launch-prefix="gdb -ex run --args" >
</node>

</launch>
<!--          launch-prefix="gdb -ex run -X-args" >-->
