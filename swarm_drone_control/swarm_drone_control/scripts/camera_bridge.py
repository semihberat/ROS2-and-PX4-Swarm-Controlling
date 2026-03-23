#!/usr/bin/env python3
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import TransitionCallbackReturn, LifecycleState
from sensor_msgs.msg import Image # Image is the message type
from custom_interfaces.msg import QRInformation
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from qreader import QReader
import json 

"""
{'qr_id': 1, 'gorev': {'formasyon': {'aktif': True, 'tip': 'OKBASI'}, 'manevra_pitch_roll': {'aktif': False, 'pitch_deg': '-10', 'roll_deg': '0'}, 'irtifa_degisim': {'aktif': True, 'deger': 20}, 'bekleme_suresi_s': 3}, 
'suruden_ayrilma': {'aktif': False, 'ayrilacak_drone_id': None, 'hedef_renk': None, 'bekleme_suresi_s': None}, 
'sonraki_qr': {'team_1': 4, 'team_2': 3, 'team_3': 5}}
"""
TEAM_NAME = "team_2"
# "ATICI_IHA"

class CameraBridge(LifecycleNode):

  def __init__(self):
    super().__init__('camera_bridge')
    self.declare_parameter("sys_id", 1)
    self.sys_id = self.get_parameter("sys_id").get_parameter_value().integer_value
    self.qreader = QReader()

    self.qr_information_publisher = self.create_publisher(QRInformation, 
                                                          f'/drone_{self.sys_id}/qr_information', 10)

    self.subscription = self.create_subscription(
      Image, 
      f'/world/aruco/model/x500_mono_cam_down_{self.sys_id}/link/camera_link/sensor/imager/image', 
      self.listener_callback, 
      10)
    
    self.timer = self.create_timer(1.0, self.timer_callback)
    self.qr_data: dict = {}
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

  def listener_callback(self, data):
    """
    Callback function.
    """
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

    decoded_text = self.qreader.detect_and_decode(image=current_frame)

    if (len(decoded_text) > 0) and (None not in decoded_text): 
      self.qr_data = self.parse_data(decoded_text)

    # Display image
    cv2.imshow(f"camera{self.sys_id}", current_frame)
    
    cv2.waitKey(1)

  def timer_callback(self):
      if not self.qr_data:
        return
      self.publish_qr_information(self.qr_data)
      self.log_qr_data()

  def publish_qr_information(self, data):
    qr_info = QRInformation()
    qr_info.qr_id = data["qr_id"] if data["qr_id"] is not None else 0
    
    qr_info.gorev.formasyon.aktif = data["gorev"]["formasyon"]["aktif"] if data["gorev"]["formasyon"]["aktif"] is not None else False
    qr_info.gorev.formasyon.tip = data["gorev"]["formasyon"]["tip"] if data["gorev"]["formasyon"]["tip"] is not None else ""
    
    qr_info.gorev.manevra_pitch_roll.aktif = data["gorev"]["manevra_pitch_roll"]["aktif"] if data["gorev"]["manevra_pitch_roll"]["aktif"] is not None else False
    qr_info.gorev.manevra_pitch_roll.pitch_deg = int(data["gorev"]["manevra_pitch_roll"]["pitch_deg"]) if data["gorev"]["manevra_pitch_roll"]["pitch_deg"] is not None else 0
    qr_info.gorev.manevra_pitch_roll.roll_deg = int(data["gorev"]["manevra_pitch_roll"]["roll_deg"]) if data["gorev"]["manevra_pitch_roll"]["roll_deg"] is not None else 0
    
    qr_info.gorev.irtifa_degisim.aktif = data["gorev"]["irtifa_degisim"]["aktif"] if data["gorev"]["irtifa_degisim"]["aktif"] is not None else False
    qr_info.gorev.irtifa_degisim.deger = int(data["gorev"]["irtifa_degisim"]["deger"]) if data["gorev"]["irtifa_degisim"]["deger"] is not None else 0
    
    qr_info.gorev.bekleme_suresi_s = int(data["gorev"]["bekleme_suresi_s"]) if data["gorev"]["bekleme_suresi_s"] is not None else 0
    
    qr_info.suruden_ayrilma.aktif = data["suruden_ayrilma"]["aktif"] if data["suruden_ayrilma"]["aktif"] is not None else False
    qr_info.suruden_ayrilma.ayrilacak_drone_id = int(data["suruden_ayrilma"]["ayrilacak_drone_id"]) if data["suruden_ayrilma"]["ayrilacak_drone_id"] is not None else 0
    qr_info.suruden_ayrilma.hedef_renk = data["suruden_ayrilma"]["hedef_renk"] if data["suruden_ayrilma"]["hedef_renk"] is not None else ""
    qr_info.suruden_ayrilma.bekleme_suresi_s = int(data["suruden_ayrilma"]["bekleme_suresi_s"]) if data["suruden_ayrilma"]["bekleme_suresi_s"] is not None else 0
    
    # parse_data'da zaten [TEAM_NAME] seçildiği için direkt data["sonraki_qr"] kullanıyoruz
    qr_info.sonraki_qr = int(data["sonraki_qr"]) if data["sonraki_qr"] is not None else 0

    self.qr_information_publisher.publish(qr_info)

  def parse_data(self, decoded_text: str): 
      parsed_data = json.loads(decoded_text[0])[0]
      return {
        "qr_id": parsed_data["qr_id"],
        "gorev": parsed_data["gorev"],
        "suruden_ayrilma": parsed_data["suruden_ayrilma"],
        "sonraki_qr": parsed_data["sonraki_qr"][TEAM_NAME]
      }
    
  def log_qr_data(self):
    if not self.qr_data:
      return
    self.get_logger().info(f"QR ID: {self.qr_data['qr_id']}")
    self.get_logger().info(f"MISSION: {self.qr_data['gorev']}")
    self.get_logger().info(f"LEAVE THE SWARM: {self.qr_data['suruden_ayrilma']}")
    self.get_logger().info(f"NEXT QR: {self.qr_data['sonraki_qr']}")
  
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