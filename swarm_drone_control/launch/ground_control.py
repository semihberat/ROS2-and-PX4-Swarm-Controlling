from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():

    number_of_drones = 5

    ld = LaunchDescription()

    pkg_share = get_package_share_directory('swarm_drone_control')
    config = os.path.join(pkg_share, 'config', 'multi_robot_params.yaml')
    
    # Load formation control nodes
    ld = load_joy_node(ld=ld)
    ld = load_formation_control(number_of_drones=number_of_drones, ld=ld, config=config)
    return ld


def load_joy_node(ld: LaunchDescription):
    joy_node = Node(
        package="joy",
        executable="joy_node",
    )
    ld.add_action(joy_node)
    return ld
# Formation Control Node
def load_formation_control(number_of_drones: int, ld: LaunchDescription, config):
    for idx in range(1, number_of_drones + 1):
        formation_control_node = Node(
            package="swarm_drone_control",
            executable="formation_control",
            name=f'formation_control_{idx}',  # ✅ Unique name
            parameters=[
                {"sys_id": idx},
            
            ],
            output='screen'  # ✅ Console output
        )       
        ld.add_action(formation_control_node)
        
    return ld