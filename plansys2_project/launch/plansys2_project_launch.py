import os
from ament_index_python.packages import get_package_share_directory
from typing import List,Dict
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('plansys2_project')
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/emergency_durative_actions.pddl',
          'namespace': namespace
          }.items())
    # Specify the actions
        
    move_agent_cmd = Node(
        package='plansys2_project',
        executable='move_agent_action_node',
        name='move_agent_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    charge_cmd = Node(
        package='plansys2_project',
        executable='charge_action_node',
        name='charge_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
    discharge_cmd = Node(
        package='plansys2_project',
        executable='discharge_action_node',
        name='discharge_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
    move_vehicle_cmd = Node(
        package='plansys2_project',
        executable='move_vehicle_action_node',
        name='move_vehicle_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
        
    give_content_cmd = Node(
        package='plansys2_project',
        executable='give_content_action_node',
        name='give_content_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])
   
    fill_cmd = Node(
        package='plansys2_project',
        executable='fill_action_node',
        name='fill_action_node',
        namespace=namespace,
        output='screen',
        parameters=[])  
        
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    # Declare the launch options
    ld.add_action(plansys2_cmd)
    ld.add_action(move_agent_cmd)
    ld.add_action(charge_cmd)
    ld.add_action(discharge_cmd)
    ld.add_action(move_vehicle_cmd)
    ld.add_action(give_content_cmd)
    ld.add_action(fill_cmd)
     
    return ld
