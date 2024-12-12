import numpy as np
import time
from math import sin, cos, pi
from matplotlib import pyplot as plt
<<<<<<< HEAD

=======
from src.guard_system.guard_system.VIPManagementSystem import VIPManagementSystem
>>>>>>> origin/main
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

class ResidentRecognitionRobot(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_resident_recognition')
        self.robot_name = robot_name
        self.get_logger().info(f"주민 인식 로봇 시작: {self.robot_name}")

        # 내비게이션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, f'{self.robot_name}/navigate_to_pose')

        # 속도 퍼블리셔
        self.vel_pub = self.create_publisher(Twist, f'/{self.robot_name}/cmd_vel', 10)

        # 카메라 이미지 구독
        self.create_subscription(
            Image,
            f'/{self.robot_name}/camera/image_raw',
            self.camera_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=ReliabilityPolicy.VOLATILE)
        )

        self.vip = VIPManagementSystem()

        # 라이다 데이터 구독
        self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=ReliabilityPolicy.VOLATILE)
        )

        # 관제탑 호출 서비스 클라이언트
        self.call_security_client = self.create_client(Trigger, f'/{self.robot_name}/call_security_robot')
        while not self.call_security_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("관제탑 호출 서비스 대기 중...")

        # 경비 로봇 도착 신호 구독
        self.create_subscription(
            Bool,
            f'/{self.robot_name}/security_arrival',
            self.security_arrival_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        )
        self.security_robot_called = False
        self.security_robot_arrived = False

        self.bridge = CvBridge()

        # 순찰 경로 설정
        self.patrol_points = [
            (0.0, 0.0),
            (2.0, 2.0),
            (-1.0, 3.0),
            (4.0, -1.0)
        ]
        self.current_patrol_index = 0

        # 순찰 타이머 시작
        self.timer = self.create_timer(1.0, self.patrol_callback)

    def patrol_callback(self):
        if self.security_robot_called and not self.security_robot_arrived:
            self.get_logger().info("경비 로봇 도착 대기 중...")
            return

        point = self.patrol_points[self.current_patrol_index]
        self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.orientation.w = 1.0

        self.nav_client.wait_for_server()
        self.get_logger().info(f"순찰 포인트로 이동 중: x={point[0]}, y={point[1]}")
        self.nav_client.send_goal_async(goal_msg)

    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_and_recognize_faces(cv_image)

    def lidar_callback(self, msg: LaserScan):
        min_range = min(msg.ranges)
        if min_range < 0.5:
            self.get_logger().warn(f"장애물 감지: 최소 거리 {min_range:.2f}m")
            self.avoid_obstacle()

    def avoid_obstacle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.vel_pub.publish(twist)
        time.sleep(1)
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)

    def detect_and_recognize_faces(self, frame):
<<<<<<< HEAD
        # 주민 데이터베이스와 매칭을 위한 SIFT 기반 이미지 매칭 수행
        for resident_name, resident_image_path in self.resident_database.items():
            self.get_logger().info(f"{resident_name}에 대한 매칭을 시도합니다.")
            match_result = self.SIFT_feature_matching(resident_image_path, frame)
            if match_result:
                self.get_logger().info(f"{resident_name} 매칭 성공!")
                return

        self.get_logger().warn("알 수 없는 사람이 감지되었습니다.")
        if not self.security_robot_called:
            self.call_security_robot()

    def SIFT_feature_matching(self, img1, img2, n=1000):
        try:
            t1 = cv2.imread(img1, 0)
            t2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)

            sift = cv2.SIFT_create()

            kp1, des1 = sift.detectAndCompute(t1, None)
            kp2, des2 = sift.detectAndCompute(t2, None)

            bf = cv2.BFMatcher()
            matches = bf.match(des1, des2)

            matches = sorted(matches, key=lambda x: x.distance)
            return len(matches[:n]) > 10  # 매칭된 포인트가 10개 이상일 경우 성공으로 간주
        except Exception as e:
            self.get_logger().error(f"SIFT 매칭 중 오류 발생: {e}")
            return False
=======
        res = self.vip.SIFT_feature_matching(frame)
        if res:
            self.call_security_robot()
>>>>>>> origin/main

    def call_security_robot(self):
        self.security_robot_called = True
        self.get_logger().info("관제탑에 경비 로봇 호출 요청 중...")
        request = Trigger.Request()
        future = self.call_security_client.call_async(request)
        future.add_done_callback(self.handle_call_security_response)

    def handle_call_security_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("경비 로봇 호출 성공!")
            else:
                self.get_logger().warn("경비 로봇 호출 실패!")
        except Exception as e:
            self.get_logger().error(f"경비 로봇 호출 중 오류 발생: {e}")

    def security_arrival_callback(self, msg):
        if msg.data:
            self.security_robot_arrived = True
            self.security_robot_called = False
            self.get_logger().info("경비 로봇 도착! 순찰을 재개합니다.")

def main(args=None):
    rclpy.init(args=args)
    robot_name = 'robot1'
    resident_robot = ResidentRecognitionRobot(robot_name)

    try:
        rclpy.spin(resident_robot)
    except KeyboardInterrupt:
        resident_robot.get_logger().info("주민 인식 로봇을 종료합니다.")
    finally:
        resident_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
