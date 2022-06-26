from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_1 = Node(
        package="nn_computer",
        executable="nn_node"
        )
    node_2 = Node(
        package="eye_cmds",
        executable="eye_node"
        )
    node_3 = Node(
        package="mouth_cmds",
        executable="mouth_node"
        )
    node_4 = Node(
        package="neck_cmds",
        executable="neck_node"
        )                
    node_5 = Node(
        package="eyeball_cmds",
        executable="eyeball_node"
        )
    node_6 = Node(
        package="mp_input",
        executable="mpinput_node"
        )

    launch_description = LaunchDescription([node_1, node_2, node_3, node_4, node_5, node_6])  # all
    # launch_description = LaunchDescription([node_1, node_2, node_3, node_6])  # expression
    # launch_description = LaunchDescription([node_4, node_5, node_6])  # tracking
    return launch_description