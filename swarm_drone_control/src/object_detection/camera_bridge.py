#!/usr/bin/env python3
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import TransitionCallbackReturn, LifecycleState
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library

# Bunu kullanacagiz https://github.com/SiliconJelly/OpenCV/tree/main/Distance%20Estimation kalibrasyondan tut hepsi icinde
class CameraBridge(LifecycleNode):

  def __init__(self):
    super().__init__('camera_bridge')
    self.declare_parameter("sys_id", 1)
    self.sys_id = self.get_parameter("sys_id").get_parameter_value().integer_value

    self.subscription = self.create_subscription(
      Image, 
      f'/world/aruco/model/x500_mono_cam_down_{self.sys_id}/link/camera_link/sensor/imager/image', 
      self.listener_callback, 
      10)
    
    self.subscription # prevent unused variable warning
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def on_configure(self, state):
    self.get_logger().info("IN ON_CONFIGURE")
    # start publisher
    # start timer
    # cancel timer
    return super().on_configure(state)
  
  def on_activate(self, state):
    self.get_logger().info("IN ON_ACTIVATE")
    # reset timer
    return super().on_activate(state)
  
  def on_deactivate(self, state):
    self.get_logger().info("IN ON_DEACTIVATE")
    # timer cancel
    return super().on_deactivate(state)

  def on_cleanup(self, state):
    self.get_logger().info("IN ON_CLEANUP")
    # destroy lifecycle publisher
    # destroy timer
    return TransitionCallbackReturn.SUCCESS
  
  def on_shutdown(self, state):
    self.get_logger().info("IN ON_SHUTDOWN")
    # destroy lifecycle publisher
    # destroy timer
    return TransitionCallbackReturn.SUCCESS
  
  def on_error(self, state):
    self.get_logger().info("IN ON_ERROR")
    return TransitionCallbackReturn.FAILURE
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)
    
    # Display image
    cv2.imshow(f"camera{self.sys_id}", current_frame)
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  camera_bridge = CameraBridge()
  
  # Spin the node so the callback function is called.
  rclpy.spin(camera_bridge)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  camera_bridge.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()