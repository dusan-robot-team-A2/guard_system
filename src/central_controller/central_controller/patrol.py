from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped
from guard_interfaces.srv import FindTarget
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .central_node import CentralNode
    
class Patrol:
    _id_counter = 0
    def __init__(self,node:"CentralNode", namespace = "gundam"):
        Patrol._id_counter += 1
        self.node:CentralNode = node
        self.patrol_id = Patrol._id_counter
        self.pose = (0,0)
        self.status = 1 #0: track, 1: patrol

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 데이터 손실 없이 안정적으로 전송
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # 구독자가 서버와 연결된 후 그 동안 수집된 데이터를 받을 수 있음
            history=QoSHistoryPolicy.KEEP_LAST, # 최근 메시지만 유지
            depth=10  # 최근 10개의 메시지를 유지
        )
        
        self.find_target_service = self.node.create_service(FindTarget, f'/{namespace}/find_target', callback=self.find_target_callback, qos_profile=self.qos_profile)
        self.patrol_toggle = self.node.create_client(SetBool, f'/{namespace}/patrol_mode')
        # self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, f'/{namespace}/amcl_pose', self.pose_sub_callback, 10)
        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_sub_callback, 10)

    def resume_patrol(self):
        request = SetBool.Request()
        request.data = True
        future = self.patrol_toggle.call_async(request)
        future.add_done_callback(self.resume_patrol_callback)
    
    def resume_patrol_callback(self,future):
        response = future.result()
        if response:
            self.node.get_logger().info('turn on patrol mode')
        else:
            self.node.get_logger().info('patrol mode service failed')

    def pose_sub_callback(self, msg:PoseWithCovarianceStamped):
        self.pose = msg.pose.pose.position.x, msg.pose.pose.position.y

    def find_target_callback(self, request:FindTarget.Request, response:FindTarget.Response):
        return self.node.find_target_callback(self.patrol_id, request, response)
    