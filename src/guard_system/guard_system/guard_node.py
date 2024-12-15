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
        self.patrol_AMR_publisher = self.create_publisher(msg.String, '/found', 10)

        # get_order_sub
        self.get_order_subscriber = ActionServer(self, MoveTo, 'get_order', self.order_callback)
        # AMR_navgoal_action_client
        self.amr_navgoal_client = ActionClient(self, NavigateToPose, '/ironman/navigate_to_pose') 

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def set_initial_pose(self, x, y, z, w):
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
    def order_callback(self, goal_handle):
        request:MoveTo.Goal = goal_handle.request

        position = request.position
        if position:
            # 목표 좌표를 PoseStamped 메시지로 생성
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'  # SLAM에서 사용되는 좌표계 (보통 'map' 프레임)
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = position.x
            goal_msg.pose.position.y = position.y
            goal_msg.pose.orientation = 1.0  # 회전 값 (회전 없음)

            if not self.amr_navgoal_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Action server not available')
                return
            

            # 현재 amr 에 입력되어 있는 명령을 무시하도록 하는 코드 필요

            goal_pos = NavigateToPose.Goal()
            goal_pos.pose = goal_msg
            future = self.amr_navgoal_client.send_goal_async(goal_pos, feedback_callback=self.feedback_callback)

            future.add_done_callback(self.goal_response_callback)

            # annotated_frame, track_ids, class_names, boxes, confidences = self.detect_objects()
            # if 'car' in class_names:
            #     self.tracking(self.frame, self.results)
            # else:
            #     move_cmd = Twist()
            #     radius = 5.0  # 원의 반지름
            #     angular_speed = 0.2  # 회전 속도
            #     linear_speed = angular_speed * radius  # 선형 속도

            #     move_cmd.linear.x = linear_speed
            #     move_cmd.angular.z = angular_speed

            #     # 일정 시간 동안 원을 그림
            #     rate = self.create_rate(10)
            #     start_time = self.get_clock().now()
            #     while (self.get_clock().now() - start_time) < Duration(seconds=2 * math.pi * radius / linear_speed):
            #         if 'car' in class_names:
            #             self.tracking(self.frame, self.results)
            #         else:
            #             self.amr_cmd_vel_pub.publish(move_cmd)
            #             rate.sleep()

            #     # 멈춤
            #     move_cmd.linear.x = 0.0
            #     move_cmd.angular.z = 0.0
            #     self.amr_cmd_vel_pub.publish(move_cmd)
        
    
    def goal_response_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            # res1 = self.vip.SIFT_feature_matching(self.image)
            res2 = MoveTo.Result()
            res2.success = True
            res2.message = '도착 완료'
            return res2
        
    def feedback_callback(self, feedback):
        # 네비게이션 피드백 처리 (필요시 사용)
        self.get_logger().info(f"Feedback: {feedback}")
    
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
