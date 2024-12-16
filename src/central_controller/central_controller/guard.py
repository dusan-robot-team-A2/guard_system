from rclpy.node import Node

from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped

class Guard:
    _id_counter = 0
    def __init__(self,node:Node, namespace = "ironman"):
        Guard._id_counter += 1
        self.node = node
        self.guard_id = Guard._id_counter
        self.pose = (0,0)
        self.status = 1 #0: stop, 1: moving
        
        # self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, f'/{namespace}/amcl_pose', self.pose_sub_callback, 10)
        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_sub_callback, 10)
    
    def pose_sub_callback(self, msg:PoseWithCovarianceStamped):
        self.pose = msg.pose.pose.position.x, msg.pose.pose.position.y
    