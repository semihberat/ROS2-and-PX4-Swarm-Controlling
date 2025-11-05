from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():

    number_of_drones = 5

    ld = LaunchDescription()

    pkg_share = get_package_share_directory('px4_ros_com')
    config = os.path.join(pkg_share, 'config', 'multi_robot_params.yaml')
    
    # Or you can load parameters for loop
    ld = load_drones(number_of_drones=number_of_drones, ld=ld, config=config)
    ld = load_cameras(number_of_cameras=number_of_drones, ld=ld, config=config)
    ld = load_path_planners(number_of_drones=number_of_drones, ld=ld, config=config)
    return ld


# MODULES TO LOAD MULTIPLE CAMERAS AND DRONES

# Path Planning Calculators
def load_path_planners(number_of_drones: int, ld: LaunchDescription, config = None):
    for idx in range(1, number_of_drones + 1):
        path_planner_node = Node(
            package="px4_ros_com",
            executable="swarm_member_path_planner",
            name=f'path_planner_{idx}',
            parameters=[
                {"sys_id": idx}
            ]
        )
        ld.add_action(path_planner_node)
    return ld

# Camera Nodes
def load_cameras(number_of_cameras: int, ld: LaunchDescription, config = None):
    for idx in range(1, number_of_cameras + 1):
        
        camera_node = Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            name = f"camera_bridge_{idx}",
            arguments=[
                f"/world/aruco/model/x500_mono_cam_down_{idx}/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image"
            ]
        )

        camera_info_node = Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            name = f"camera_info_bridge_{idx}",
            arguments=[
                f"/world/aruco/model/x500_mono_cam_down_{idx}/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo"
            ]
        )
        ld.add_action(camera_node)
        ld.add_action(camera_info_node)
    return ld

# Drone Nodes
def load_drones(number_of_drones: int, ld: LaunchDescription, config = None):
    for idx in range(1, number_of_drones + 1):
        drone_node = Node(
            package='px4_ros_com',
            executable='uav_controller',
            name=f'drone{idx}',
            parameters=[config, 
                        {'sys_id': idx},
                        {'number_of_drones': number_of_drones}
                        ]
        )
        ld.add_action(drone_node)

    return ld