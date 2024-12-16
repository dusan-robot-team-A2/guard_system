import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import time

from guard_interfaces.srv import FindTarget
from guard_interfaces.msg import Target
from guard_interfaces.action import MoveTo
from geometry_msgs.msg import Point

class UnitTest(Node):
    def __init__(self):
        super().__init__('unit_test_node')

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,  # 데이터 손실 없이 안정적으로 전송
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, # 구독자가 서버와 연결된 후 그 동안 수집된 데이터를 받을 수 있음
            history=QoSHistoryPolicy.KEEP_LAST, # 최근 메시지만 유지
            depth=10  # 최근 10개의 메시지를 유지
        )

        self.get_logger().info("central node init")

        self.find_target_client = self.create_client(FindTarget, 'find_target',qos_profile=self.qos_profile)
        # self.find_target_service = self.create_service(FindTarget, 'find_target', callback=self.find_target_callback, qos_profile=self.qos_profile)
        # self.command_action_server = ActionServer(self, MoveTo, 'get_order', self.command_action_callback)


        # 서비스가 준비될 때까지 대기
        while not self.find_target_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        self.timer = self.create_timer(10.0, self.timer_callback)
    
    def command_action_callback(self, goal_handle):
        self.get_logger().info("test node action init")

        reqeust:MoveTo.Goal = goal_handle.request
        self.get_logger().info(f"moving to {reqeust.position.x}, {reqeust.position.y}")
        time.sleep(5)

        result = MoveTo.Result()
        result.success = True
        result.message = "test node action success"
        goal_handle.succeed()
        self.get_logger().info("test node action end")

        return result

    def timer_callback(self):
        self.get_logger().info("Sending request to the server...")

        req = FindTarget.Request()
        req.patrol_id = 1

        point = Point()
        point.x = 2.0
        point.y = -1.0
        
        target = Target()
        target.object_id = 1
        target.class_name = "test class"
        target.object_position = point
        objects = [target,]

        req.objects = objects

        future = self.find_target_client.call_async(req)
        future.add_done_callback(self.handle_response)
    
    def handle_response(self, future):
        response = future.result()
        self.get_logger().info(f"Result: {response.message}")
            

        
            
                




    
    
    

    
def main(args=None):
    rclpy.init(args=args)
    node = UnitTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()