import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import time
from enum import Enum
from types import SimpleNamespace

from guard_interfaces.srv import FindTarget

class RobotStatus(Enum):
    IDLE = 0
    MOVING = 1
    WORKING = 2
    ERROR = 3

class CentralNode(Node):
    def __init__(self):
        super().__init__('central_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 데이터 손실 없이 안정적으로 전송
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # 구독자가 서버와 연결된 후 그 동안 수집된 데이터를 받을 수 있음
            history=QoSHistoryPolicy.KEEP_LAST, # 최근 메시지만 유지
            depth=10  # 최근 10개의 메시지를 유지
        )

        self.get_logger().info("central node init")

        self.find_target_service = self.create_service(FindTarget, 'find_target', callback=self.find_target_callback, qos_profile=self.qos_profile)
        

        self.targets = {}
        self.patrol = SimpleNamespace( pose=None, status=-1, )
        self.guardian = SimpleNamespace( pose=None, status=-1,)
        
        # status:-1 not connected, status:0 대기중, status:1 순찰 중, status:2 수상자 찾음,  
        self.guardians = {}

    def init_status(self):
        self.targets = {}
        self.patrols = {}
    
    def find_target_callback(self, request:FindTarget.Request, response:FindTarget.Response):
        stamp = request.stamp
        targets = request.objects

        for target in targets:
            if target

        self.get_logger().info("find target callback")

    
    
    

    
def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()