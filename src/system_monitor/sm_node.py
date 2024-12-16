import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from PySide2.QtCore import QThread, Signal
import json

from nav_msgs.msg import OccupancyGrid


class SystemMonitoringNode(QThread, Node):
    Signal = Signal(list)
    
    def __init__(self):
        super().__init__('data_subscriber')

        self.map_subscribtion = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.system_info_subscribtion = self.create_subscription(
            String,
            '/system_info',
            self.system_info_callback,
            10
        )

        self.map_data = None
        self.system_info_str = None

    def map_callback(self, msg:OccupancyGrid):
        self.map_data = msg

    def system_info_callback(self, msg:String):
        self.system_info_str = msg.data

    
