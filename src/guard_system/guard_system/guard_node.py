import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import rclpy.time
import std_msgs.msg as msg
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PoseStamped, Twist, Quaternion, Pose
from nav2_msgs.srv import SetInitialPose
from cv_bridge import CvBridge
import numpy as np
import cv2
from nav2_msgs.action import NavigateToPose
import math
import tf2_ros

class MoveToZoneActionServer(Node):

    def __init__(self):
        super().__init__('move_to_zone_action_server')

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
        self.AMR_image_subscriber = self.create_subscription(Image,'/tb3_0/camera/image_raw',10)
        # SM_tracked_image_pub
        self.sm_tracked_image_publisher = self.create_publisher(Image, 'tracked_image', 10)
        # patrol_AMR_pub
        self.patrol_AMR_publisher = self.create_publisher(msg.String, 'patrol', 10)

        # get_order_sub
        self.get_order_subscriber = self.create_subscription(msg.String, 'get_order', 10, self.order_callback)
        # AMR_navgoal_action_client
        self.amr_navgoal_client = ActionClient(self, NavigateToPose, 'navigate_to_pose') 

        self.img_timer = self.create_timer(0.1, self.image_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(1.0, self.tf_timer_callback)

        
        # 5개 영역의 하나의 꼭짓점 좌표 정의 (각 영역별로 임의의 path를 지정해주는 로직)
        self.home_pos = Pose(position = Point(x=0.043317, y=0.033049), orientation = self.euler_to_quaternion(0, 0, -0.003))

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Convert Euler angles to a quaternion
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)
    
    def tf_timer_callback(self):
        try:
            # 첫 번째 로봇 ('tb3_0')의 'base_link'에서 'map' 좌표를 얻기
            transform_tb3_0 = self.tf_buffer.lookup_transform('map', 'tb3_0/base_link', rclpy.time.Time())

            # 첫 번째 로봇 (tb3_0) 좌표 출력
            x_0 = transform_tb3_0.transform.translation.x
            y_0 = transform_tb3_0.transform.translation.y
            z_0 = transform_tb3_0.transform.translation.z
            return x_0, y_0, z_0

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"좌표를 얻는 데 실패: {e}")

    # 디지털 맵 내 지정 구역으로 이동
    def order_callback(self, msg):
        
        if msg == '1':
            x, y, z = self.tf_timer_callback()
            # 목표 좌표를 PoseStamped 메시지로 생성
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'  # SLAM에서 사용되는 좌표계 (보통 'map' 프레임)
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.orientation = 1.0  # 회전 값 (회전 없음)

            if not self.amr_navgoal_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Action server not available')
                return
            

            # 현재 amr 에 입력되어 있는 명령을 무시하도록 하는 코드 필요

            goal_pos = NavigateToPose.Goal()
            goal_pos.pose = goal_msg
            self.amr_navgoal_client.send_goal_async(goal_pos, feedback_callback=self.feedback_callback)            

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
        else:
            # 목표 좌표를 PoseStamped 메시지로 생성
            goal_msg = PoseStamped()
            goal_msg.header.frame_id = 'map'  # SLAM에서 사용되는 좌표계 (보통 'map' 프레임)
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.position.x = self.home_pos.position.x
            goal_msg.pose.position.y = self.home_pos.position.y
            goal_msg.pose.orientation = self.home_pos.orientation  # 회전 값 (회전 없음)

            if not self.amr_navgoal_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().info('Action server not available')
                return
            

            # 현재 amr 에 입력되어 있는 명령을 무시하도록 하는 코드 필요

            goal_pos = NavigateToPose.Goal()
            goal_pos.pose = goal_msg
            self.amr_navgoal_client.send_goal_async(goal_pos, feedback_callback=self.feedback_callback)
        
    def feedback_callback(self, feedback):
        # 네비게이션 피드백 처리 (필요시 사용)
        self.get_logger().info(f"Feedback: {feedback}")
    
    def convert_ros_to_cv2_image(self, ros_image):
        # ROS2 이미지를 OpenCV로 변환
        bridge = CvBridge()
        return bridge.imgmsg_to_cv2(ros_image, encoding='bgr8')

    def image_callback(self):
        # 객체 감지 및 추적
        try:
            detections, track_ids, class_names, boxes, confidences = self.detect_objects(self.frame, self.self.results)
        except:
            return False
        
        # tracking
        self.tracking()

        ros_image = self.convert_cv2_to_ros_image(detections)
        # ros_image.header = msg.Header()
        # ros_image.header.stamp = self.get_clock().now().to_msg()
        self.AMR_image_publisher.publish(ros_image)
        self.get_logger().info("Publishing video frame")

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
                self.get_logger().info("[INFO] Initial pose set successfully")  #send ui
                self.initial_status = True
            else:
                self.get_logger().warn("[WARN] Failed to set initial pose")
        except Exception as e:
            self.get_logger().error(f"[ERROR] Service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    action_server = MoveToZoneActionServer()
    rclpy.spin(action_server)  # 노드가 계속 실행되도록
    action_server.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
