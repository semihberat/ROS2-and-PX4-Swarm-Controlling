from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os 

def generate_launch_description():

    number_of_drones = 5

    ld = LaunchDescription()

    # Or you can load parameters for loop
    ld = load_drones(number_of_drones=number_of_drones, ld=ld)
    ld = load_cameras(number_of_cameras=1, ld=ld)
    ld = load_swarm_communication(number_of_drones=number_of_drones, ld=ld)
    ld = load_path_planners(number_of_drones=number_of_drones, ld=ld)
    ld = load_camera_processes(number_of_cameras=1, ld=ld)
    ld = load_gamepad_controllers(number_of_drones=number_of_drones, ld=ld)
    return ld

def load_gamepad_controllers(number_of_drones: int, ld: LaunchDescription):
    for idx in range(1, number_of_drones + 1):
        gamepad_controller_node = LifecycleNode(
            package="swarm_drone_control",
            executable="gamepad_controller",
            namespace="",
            name=f'gamepad_controller_{idx}',
            parameters=[
                {"sys_id": idx}
            ]
        )
        ld.add_action(gamepad_controller_node)
    return ld

# Camera Process Nodes
def load_camera_processes(number_of_cameras: int, ld: LaunchDescription):
    for idx in range(1, number_of_cameras + 1):
        camera_process_node = LifecycleNode(
            package="swarm_drone_control",
            executable="camera_bridge.py",
            namespace="",
            name=f'camera_process_{idx}',
            parameters=[
                {"sys_id": idx}
            ]
        )
        ld.add_action(camera_process_node)
    return ld

# Path Planning Calculators

def load_swarm_communication(number_of_drones: int, ld: LaunchDescription):
    for idx in range(1, number_of_drones + 1):
        swarm_communication_node = Node(
            package="swarm_drone_control",
            executable="swarm_communication",
            name=f'swarm_communication_{idx}',
            parameters=[
                {"sys_id": idx},
                {"number_of_drones": number_of_drones}
            ]
        )
        ld.add_action(swarm_communication_node)
    return ld

def load_path_planners(number_of_drones: int, ld: LaunchDescription):
    for idx in range(1, number_of_drones + 1):
        path_planner_node = LifecycleNode(
            package="swarm_drone_control",
            namespace="",
            executable="swarm_member_path_planner",
            name=f'path_planner_{idx}',
            parameters=[
                {"sys_id": idx}
            ]
        )
        ld.add_action(path_planner_node)
    return ld

# Camera Nodes
def load_cameras(number_of_cameras: int, ld: LaunchDescription):
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
def load_drones(number_of_drones: int, ld: LaunchDescription):
    for idx in range(1, number_of_drones + 1):
        drone_node = Node(
            package='swarm_drone_control',
            executable='uav_controller',
            name=f'drone{idx}',
            parameters=[
                        {'sys_id': idx},
                        ]
        )
        ld.add_action(drone_node)

    return ld