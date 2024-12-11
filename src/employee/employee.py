import os
import cv2
import numpy as np
import time
from math import sin, cos, pi

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
import dlib

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
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        )

        # 라이다 데이터 구독
        self.create_subscription(
            LaserScan,
            f'/{self.robot_name}/scan',
            self.lidar_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
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
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=ReliabilityPolicy.VOLATILE)
        )
        self.security_robot_called = False
        self.security_robot_arrived = False

        # 주민 데이터베이스
        self.resident_database = {
            'John Doe': 'face_data_1.npy',
            'Jane Doe': 'face_data_2.npy'
        }
        self.bridge = CvBridge()

        # 얼굴 탐지 및 인코딩 모델 초기화
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
        self.face_rec_model = dlib.face_recognition_model_v1("dlib_face_recognition_resnet_model_v1.dat")

        # 주민 얼굴 인코딩 데이터 로드
        self.known_face_encodings = []
        self.known_face_names = []

        for name, file in self.resident_database.items():
            face_encoding = np.load(file)
            self.known_face_encodings.append(face_encoding)
            self.known_face_names.append(name)

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
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = self.detect_faces(rgb_frame)
        face_encodings = self.encode_faces(rgb_frame, face_locations)

        for face_encoding, face_location in zip(face_encodings, face_locations):
            matches = self.compare_faces(self.known_face_encodings, face_encoding)
            name = "Unknown"

            face_distances = self.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]

            self.display_face(frame, face_location, name)

            if name != "Unknown":
                self.get_logger().info(f"주민 확인: {name}. 환영합니다!")
            else:
                self.get_logger().warn("알 수 없는 사람을 감지했습니다.")
                if not self.security_robot_called:
                    self.call_security_robot()

        cv2.imshow("Resident Recognition", frame)
        cv2.waitKey(1)

    def detect_faces(self, rgb_frame):
        return self.detector(rgb_frame, 1)

    def encode_faces(self, rgb_frame, face_locations):
        encodings = []
        for face in face_locations:
            shape = self.predictor(rgb_frame, face)
            encoding = np.array(self.face_rec_model.compute_face_descriptor(rgb_frame, shape))
            encodings.append(encoding)
        return encodings

    def compare_faces(self, known_encodings, face_encoding, tolerance=0.6):
        distances = self.face_distance(known_encodings, face_encoding)
        return distances <= tolerance

    def face_distance(self, known_encodings, face_encoding):
        return np.linalg.norm(known_encodings - face_encoding, axis=1)

    def display_face(self, frame, face_location, name):
        top, right, bottom, left = face_location.top(), face_location.right(), face_location.bottom(), face_location.left()
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        cv2.putText(frame, name, (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

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
