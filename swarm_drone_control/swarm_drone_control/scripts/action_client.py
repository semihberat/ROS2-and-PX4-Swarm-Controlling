#!/usr/bin/env python3
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus
from custom_interfaces.action import MoveDrone
from custom_interfaces.msg import GeoPoint, Formation
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

class DroneClients(Node):
    def __init__(self, sys_id:int = 1):
        super().__init__(f"client_{sys_id}")
        self.sys_id = sys_id
        self.get_logger().info(f"Action client for drone {self.sys_id} initialized")
        self.cb_group_ = MutuallyExclusiveCallbackGroup()
        self.action_client_ = ActionClient(self, MoveDrone, f"/uav_{self.sys_id}/move_drone")

    def send_goal(self, command:int, geo_points:list[GeoPoint], formation:Formation ):
        self.action_client_.wait_for_server()
        goal = MoveDrone.Goal()
        goal.command = command
        goal.geo_points = geo_points
        goal.formation = formation

        self.get_logger().info(f"Sending goal to drone {self.sys_id} with command {command}, \
                               geo_points {geo_points} and formation {formation}")
        
        self.action_client_.send_goal_async(goal, feedback_callback = self.feedback_callback). \
            add_done_callback(self.goal_response_callback)
        
        
    def cancel_goal(self):
        self.get_logger().info(f"Cancelling goal for drone {self.sys_id}")
        self.goal_handle_.cancel_goal_async()
  

    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info(f"Goal accepted for drone {self.sys_id}")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().info(f"Goal rejected for drone {self.sys_id}")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal succeeded for drone {self.sys_id} with result {result}")
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f"Goal canceled for drone {self.sys_id} with result {result}")
        else: 
            self.get_logger().info(f"Goal failed for drone {self.sys_id} with result {result}")

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback for drone {self.sys_id} with feedback {feedback}")

class GroundControlUI():
    def __init__(self, number_of_drones: int = 3):
        rclpy.init()
        self.number_of_drones = number_of_drones
        self.executor = SingleThreadedExecutor()
        self.nodes = []
        for i in range(1, self.number_of_drones + 1):
            node = DroneClients(sys_id=i)
            self.nodes.append(node)
            self.executor.add_node(node)


    def send_swarm_goal(self, command:int, geo_points:list[GeoPoint], formation:Formation, exc: list[int]):
        for i in range(self.number_of_drones + 1):
            if i not in exc:
                self.nodes[i-1].send_goal(command=command, geo_points=geo_points, formation=formation)
        


def main(args=None):
    gcUI = GroundControlUI(number_of_drones=3)
    gcUI.send_swarm_goal(command=1, geo_points=[GeoPoint()], formation = Formation())


if __name__ == "__main__":
    main()