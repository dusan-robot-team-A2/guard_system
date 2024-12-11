import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import time 


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

        self.order_service = self.create_service(P,'/order', qos_profile=self.qos_profile, callback=self.order_callback)
        self.log_sub = self.create_subscription(Log, '/rosout', self.log_callback, self.qos_profile)

        self.Nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.kitchen_display_logger = self.get_logger().get_child("kitchen_display")
        
        # 사전 정의된 목표 좌표
        self.set_nav2_pose = {
            '0': Point(x=0.0, y=0.0, z=0.0),  # 복귀 지점
            '1': Point(x=2.5, y=1.5, z=0.0),  # table_id = '1'
            '2': Point(x=2.5, y=0.5, z=0.0),   # table_id = '2'
            '3': Point(x=2.5, y=-0.6, z=0.0),   # table_id = '3'
            '4': Point(x=1.45, y=1.5, z=0.0),   # table_id = '4'
            '5': Point(x=1.45, y=0.5, z=0.0),   # table_id = '5'
            '6': Point(x=1.45, y=-0.5, z=0.0),   # table_id = '6'
            '7': Point(x=0.3, y=1.5, z=0.0),   # table_id = '7'
            '8': Point(x=0.3, y=0.5, z=0.0),   # table_id = '8'
            '9': Point(x=0.3, y=-0.5, z=0.0),   # table_id = '9'
        }
        self.current_goal = None  # 현재 목표 ID

    
def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()