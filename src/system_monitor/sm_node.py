import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import json

from nav_msgs.msg import OccupancyGrid


class SystemMonitoringNode(Node):
    def __init__(self):
        super().__init__('data_subscriber')

        self.map_subscribtion = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.map_data = None

    def map_callback(self, msg:OccupancyGrid):
        self.map_data = msg

    
