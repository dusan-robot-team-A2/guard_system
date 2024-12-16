import rclpy
from rclpy.action import ActionClient, ActionServer
from rclpy.node import Node
import rclpy.time
import std_msgs.msg as msg
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PoseStamped, Twist, Quaternion, Pose
from nav2_msgs.srv import SetInitialPose
from cv_bridge import CvBridge
import cv2
from .VIPManagementSystem import VIPManagementSystem
from nav2_msgs.action import NavigateToPose
import math
from guard_interfaces.action import MoveTo

class Guard_node(Node):

    def __init__(self):
        super().__init__('guard_node')
        self.get_logger().info("init guard node")

        self.vip = VIPManagementSystem()

        self.num = 3
        self.init_pose = [0.0, 0.0, 0.0, 1.0]
        self.goal_poses = [[0.0, 0.0] for _ in range(self.num)]
        self.setting_poses = [False for _ in range(self.num)]

        # SetInitialPose 서비스 클라이언트 생성
        self.set_initial_pose_service_client = self.create_client(
            SetInitialPose,
            '/set_initial_pose'
        )

        self.initial_status = False

        self.set_initial_pose(*self.init_pose)

        # GUARD AMR_Image sub
        self.AMR_image_subscriber = self.create_subscription(Image,'/ironman/camera/image_raw',self.image_callback, 10)
        # SM_tracked_image_pub
        self.sm_tracked_image_publisher = self.create_publisher(Image, '/tracked_image', 10)
        # patrol_AMR_pub
        # self.patrol_AMR_publisher = self.create_publisher(msg.String, '/found', 10)

        # get_order_sub
        self.get_order_subscriber = ActionServer(self, MoveTo, 'get_order', self.order_callback)
        # AMR_navgoal_action_client
        # self.amr_navgoal_client = ActionClient(self, NavigateToPose, '/ironman/navigate_to_pose') 
        self.amr_navgoal_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def set_initial_pose(self, x, y, z, w):
        self.get_logger().info("set initial pose")
        req = SetInitialPose.Request()
        req.pose.header.frame_id = 'map'
        req.pose.pose.pose.position = Point(x=x, y=y, z=0.0)
        req.pose.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=z, w=w)
        req.pose.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.1,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.01, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.01, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.01
        ]

        future = self.set_initial_pose_service_client.call_async(req)
        future.add_done_callback(self.handle_initial_pose_response)

    def handle_initial_pose_response(self, future):
        try:
            response = future.result()
            if response:
                self.initial_status = True
            else:
                self.get_logger().warn("[WARN] Failed to set initial pose")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Service call failed: {e}")

    # 디지털 맵 내 지정 구역으로 이동
    async def order_callback(self, goal_handle):
        request:MoveTo.Goal = goal_handle.request

        position = request.position
        if position:
            # 목표 좌표를 PoseStamped 메시지로 생성
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'  # SLAM에서 사용되는 좌표계 (보통 'map' 프레임)
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = position.x
            goal_msg.pose.position.y = position.y
            goal_msg.pose.orientation.w = 1.0  # 회전 값 (회전 없음)

            if not self.amr_navgoal_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Action server not available')
                return
            
        # NavigateToPose 액션 실행
        nav_result = await self.navigate_to_pose(goal_msg)

        # 결과 처리
        if nav_result:
            self.get_logger().info("Navigation succeeded.")
            goal_handle.succeed()
            return MoveTo.Result(success=True)
        else:
            self.get_logger().info("Navigation failed.")
            goal_handle.abort()
            return MoveTo.Result(success=False)

        
    async def navigate_to_pose(self, target_pose):
        """NavigateToPose 액션 클라이언트를 통해 목표로 이동."""
        self.get_logger().info("Waiting for NavigateToPose Action Server...")
        self.amr_navgoal_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = target_pose

        self.get_logger().info(f"Sending navigation goal to {target_pose.pose.position.x}, {target_pose.pose.position.y}")
        send_goal_future = self.amr_navgoal_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().info("Navigation goal rejected.")
            return False

        self.get_logger().info("Navigation goal accepted.")
        get_result_future = goal_handle.get_result_async()
        result = await get_result_future

        if result.status == 4:  # STATUS_SUCCEEDED
            return True
        else:
            return False    
    
    def convert_ros_to_cv2_image(self, ros_image):
        # ROS2 이미지를 OpenCV로 변환
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(ros_image, encoding='bgr8')

    def image_callback(self, image):
        image = self.convert_ros_to_cv2_image(image)
        self.image = image
        

    def handle_initial_pose_response(self, future):
        try:
            response = future.result()
            if response:
                self.get_logger().info("[INFO] Initial pose set successfully")  #send ui
                self.initial_status = True
            else:
                self.get_logger().warn("[WARN] Failed to set initial pose")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Guard_node()
    rclpy.spin(node)  # 노드가 계속 실행되도록
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
