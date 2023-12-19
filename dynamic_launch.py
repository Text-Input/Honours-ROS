from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

# List of parameters to pass to the nodes, along with the default value
parameters = [
    ("dalg", "minimize_time_v2"),
    ("salg", "greedy"),
    ("known_target_percentage", "0.5")
]

def generate_launch_description():
    args = [DeclareLaunchArgument(x[0], default_value=x[1]) for x in parameters]

    args_values = [{x[0]: LaunchConfiguration(x[0])} for x in parameters]
    dynamic_tasks_node = Node(package='dynamic_tasks', executable='dynamic_tasks', parameters=args_values)
    analysis_node = Node(package='analysis', executable='talker', parameters=args_values)

    return LaunchDescription([
        *args,
        analysis_node,
        dynamic_tasks_node
    ])
