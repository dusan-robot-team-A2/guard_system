import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import time
from enum import Enum
from types import SimpleNamespace
<<<<<<< HEAD

from guard_interfaces.srv import FindTarget

class RobotStatus(Enum):
    IDLE = 0
    MOVING = 1
    WORKING = 2
    ERROR = 3
=======
import threading
import asyncio

from guard_interfaces.srv import FindTarget
from guard_interfaces.msg import Target
from guard_interfaces.action import MoveTo

class RobotStatus(Enum):
    STANDBY = 0
    PATROL = 1
    MOVING = 2
>>>>>>> feat/central_controller

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
        self.command_action = ActionClient(self, MoveTo, 'get_order')

        self.targets:dict[int,Target] = {}
        self.target_order:list[int] = [] # target 탐색 순서
        self.patrol = SimpleNamespace( pose=None, status= RobotStatus.PATROL, )
        self.guardian = SimpleNamespace( pose=None, status=RobotStatus.STANDBY,)

        self.command_thread = None
    
    def find_target_callback(self, request:FindTarget.Request, response:FindTarget.Response):
        stamp = request.stamp
        targets:list[Target] = request.objects

        self.get_logger().info("Requested find target")

        for target in targets:
            object_id = target.object_id    
            
            if object_id not in self.targets:
                self.target_order.append(object_id)

            self.targets[object_id] = target
        
        self.run_command_thread()

        response.operation_successful = True
        response.message = "successful"
        return response
            
    def run_command_thread(self):
        if self.command_thread is None or not self.command_thread.is_alive():  # 스레드가 없거나 종료된 경우
            self.command_thread = threading.Thread(target=self.run_event_loop_in_thread)
            self.command_thread.start()
        return
    
    def run_event_loop_in_thread(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        loop.run_until_complete(self.command_find())
        
    async def command_find(self):
        while self.target_order:
            self.get_logger().info(str(self.target_order))
            self.get_logger().info(str(self.targets))
            target_id = self.target_order.pop(0)
            target = self.targets[target_id]

            # self.get_logger().info(f"command to go x:{target.object_position.x}, y:{target.object_position.y}")
            # time.sleep(5)
            goal_msg = MoveTo.Goal()
            goal_msg.position = target.object_position

            self.get_logger().info("Waiting for action server...")
            self.command_action.wait_for_server()

            self.get_logger().info("Sending goal...")
            send_goal_future = self.command_action.send_goal_async(goal_msg)
            goal_handle = await send_goal_future

            if not goal_handle.accepted:
                self.get_logger().info("Goal was rejected.")
                self.target_order.insert(0, target_id)
                continue

            self.get_logger().info("Goal accepted. Waiting for result...")

            result_future = goal_handle.get_result_async()
            result_handle = await result_future
            result = result_handle.result
            self.get_logger().info(f"Result: success={result.success}, message='{result.message}'")

            if result.success:
                del self.targets[target_id]
            else:
                self.get_logger().warn(f"Failed to detect") # 실패할 경우 어떻게 할지 정해야함 target에 추가할 것인지 아닌지


                
    

    
def main(args=None):
    rclpy.init(args=args)
    node = CentralNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()