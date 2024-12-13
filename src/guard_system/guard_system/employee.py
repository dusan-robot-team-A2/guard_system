import time
from .VIPManagementSystem import VIPManagementSystem
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from std_msgs.msg import Bool

from guard_interfaces.srv import FindTarget
from guard_interfaces.msg import Target
from geometry_msgs.msg import Point

class ResidentRecognitionRobot(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_resident_recognition')
        self.robot_name = robot_name
        self.get_logger().info(f"주민 인식 로봇 시작: {self.robot_name}")

        # 내비게이션 클라이언트
        self.nav_client = ActionClient(self, NavigateToPose, f'{self.robot_name}/navigate_to_pose')

        # 카메라 이미지 구독
        self.create_subscription(
            Image,
            f'/{self.robot_name}/camera/image_raw',
            self.camera_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=ReliabilityPolicy.VOLATILE)
        )

        self.vip = VIPManagementSystem()

        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.call_security_client = self.create_client(FindTarget, 'find_target', qos_profile=self.qos_profile)
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

        self.bridge = CvBridge()

        # 순찰 경로 설정
        self.patrol_points = [
            (0.0, 0.0),
            (2.0, 2.0),
            (-1.0, 3.0),
            (4.0, -1.0)
        ]
        self.current_patrol_index = 0
        self.navigation_in_progress = False
        self.stopped_due_to_intruder = False

        self.send_to_next_waypoint()

    def send_to_next_waypoint(self):
        if self.navigation_in_progress:
            self.get_logger().info("현재 내비게이션이 진행 중입니다.")
            return

        if self.stopped_due_to_intruder:
            self.get_logger().info("침입자 감지로 인해 대기 중입니다.")
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
        self.get_logger().info(f"웨이포인트로 이동 중: x={point[0]}, y={point[1]}")
        self.navigation_in_progress = True
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.navigation_done_callback)

    def navigation_done_callback(self, future):
        try:
            result = future.result()
            if result.status == 4:  # STATUS_SUCCEEDED
                self.get_logger().info("목표 지점에 도착했습니다.")
            else:
                self.get_logger().warn(f"내비게이션 실패: 상태 {result.status}")
        except Exception as e:
            self.get_logger().error(f"내비게이션 중 오류 발생: {e}")
        finally:
            self.navigation_in_progress = False
            self.send_to_next_waypoint()  # 다음 웨이포인트로 이동

    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_and_recognize_faces(cv_image)

    def lidar_callback(self, msg: LaserScan):
        min_range = min(msg.ranges)
        if min_range < 0.5:
            self.get_logger().warn(f"장애물 감지: 최소 거리 {min_range:.2f}m")
            self.avoid_obstacle()

    def detect_and_recognize_faces(self, frame):
        res = self.vip.SIFT_feature_matching(frame)
        if res:
            self.get_logger().info("외부인 발견")
            self.stop_and_call_security_robot()

    def stop_and_call_security_robot(self):
        if self.security_robot_called:
            self.get_logger().info("이미 경비 로봇을 호출했습니다.")
            return

        self.security_robot_called = True
        self.stopped_due_to_intruder = True
        self.get_logger().info("관제탑에 경비 로봇 호출 요청 중...")

        request = FindTarget.Request()
        point = Point()
        point.x = 100.0
        point.y = 100.0

        target = Target()
        target.object_id = 1
        target.class_name = "Intruder"
        target.object_position = point
        request.objects = [target]

        future = self.call_security_client.call_async(request)
        future.add_done_callback(self.handle_call_security_response)

    def handle_call_security_response(self, future):
        try:
            response: FindTarget.Response = future.result()
            if response.operation_successful:
                self.get_logger().info("경비 로봇 호출 성공!")
            else:
                self.get_logger().warn("경비 로봇 호출 실패!")
        except Exception as e:
            self.get_logger().error(f"경비 로봇 호출 중 오류 발생: {e}")

    def security_arrival_callback(self, msg):
        if msg.data:
            self.get_logger().info("경비 로봇 도착! 순찰을 재개합니다.")
            self.security_robot_arrived = True
            self.security_robot_called = False
            self.stopped_due_to_intruder = False
            self.send_to_next_waypoint()

    def avoid_obstacle(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.vel_pub.publish(twist)
        time.sleep(1)
        twist.angular.z = 0.0
        self.vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    robot_name = 'gundam'
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