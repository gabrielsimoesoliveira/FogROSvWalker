# ~/fog_ws/src/my_turtlesim_control/launch/turtlesim_local.launch.py
# nó de controle rodando localmente

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # nó do turtlesim
    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node",
        output="screen"
    )

    # nó de controle
    automatic_control_node = Node(
        package="my_turtlesim_control",
        executable="automatic_control_node", # Nome do executável definido no setup.py
        output="screen"
    )

    ld.add_action(turtlesim_node)
    ld.add_action(automatic_control_node)
    
    return ld
