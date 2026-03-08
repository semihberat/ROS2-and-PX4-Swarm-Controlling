import rclpy
from rclpy.lifecycle.node import TransitionCallbackReturn

class CameraBridgeLifecycleMixin:
  """
  Mixin class for providing lifecycle state transitions.
  This allows separating lifecycle logic from the main Node implementation,
  similar to separating into multiple .cpp files in C++, but following Pythonic mixin patterns.
  """
  
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
