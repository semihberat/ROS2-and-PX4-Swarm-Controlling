from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():

    number_of_drones = 5

    ld = LaunchDescription()

    pkg_share = get_package_share_directory('swarm_drone_control')
    config = os.path.join(pkg_share, 'config', 'multi_robot_params.yaml')
    
    ld = change_state_clients(number_of_drones=number_of_drones, ld=ld, config=None)
    return ld

def change_state_clients(number_of_drones: int, ld: LaunchDescription, config = None):
    for idx in range(1, number_of_drones + 1):
        change_state_node = Node(
            package="swarm_drone_control",
            executable="change_state",
            name=f'change_state_client_{idx}',
            parameters=[
                {"sys_id": idx}
            ]
        )
        ld.add_action(change_state_node)
    return ld

