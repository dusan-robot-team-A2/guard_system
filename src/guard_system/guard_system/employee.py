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
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Bool

from guard_interfaces.srv import FindTarget
from guard_interfaces.msg import Target
from geometry_msgs.msg import Point

class ResidentRecognitionRobot(Node):
    def __init__(self, robot_name):
        super().__init__(f'{robot_name}_resident_recognition')
        self.robot_name = robot_name
        self.patrol_id = 1

        self.get_logger().info(f"주민 인식 로봇 시작: {self.robot_name}")

        # 내비게이션 클라이언트
        # self.nav_client = ActionClient(self, NavigateToPose, f'/{self.robot_name}/navigate_to_pose')
        self.nav_client = ActionClient(self, NavigateToPose, f'/navigate_to_pose')
        self.nav_goal_handle = None

        # 카메라 이미지 구독
        self.create_subscription(
            Image,
            # f'/{self.robot_name}/camera/image_raw',
            f'/camera/image_raw',
            self.camera_callback,
            10
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
        # self.create_subscription(
        #     Bool,
        #     f'/{self.robot_name}/security_arrival',
        #     self.security_arrival_callback,
        #     QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=ReliabilityPolicy.VOLATILE)
        # )

        self.create_service(SetBool, f'/{self.robot_name}/patrol_mode', self.patrol_mode_callback)
        # self.cmd_vel_pub = self.create_publisher(Twist, '/{self.robot_name}/cmd_vel', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.security_robot_called = False

        self.bridge = CvBridge()

        # 순찰 경로 설정
        self.patrol_points = [
            (0.0, 0.0),
            (1.0, -1.0),
            (3.75, 0.77),
            (1.6, 2.26)
        ]
        self.current_patrol_index = 0
        self.navigation_in_progress = False
        self.stopped_due_to_intruder = False

        self.timer = self.create_timer(10, self.timer_callback)

        self.send_to_next_waypoint()

    def timer_callback(self):
        self.stop_and_call_security_robot()
    
    def patrol_mode_callback(self, request:SetBool.Request, response:SetBool.Response):
        # 요청이 들어오면 상태를 반전시킴
        if request.data:
            self.get_logger().info(f"Patrol mode on")
            self.patrol_start()
        else:
            self.cancel_goal()
            self.get_logger().info(f"Track mode on")

        response.success = True
        return response

    def send_to_next_waypoint(self):
        if self.navigation_in_progress:
            self.get_logger().info("현재 내비게이션이 진행 중입니다.")
            return

        if self.stopped_due_to_intruder:
            self.get_logger().info("침입자 감지로 인해 대기 중입니다.")
            return

        point = self.patrol_points[self.current_patrol_index]

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
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Nav Goal rejected')
            return
        self.get_logger().info('Nav Goal accepted')
        self.nav_goal_handle = goal_handle
        # 목표 완료 후 결과 기다리기
        goal_handle.get_result_async().add_done_callback(self.navigation_result_callback)


    def navigation_result_callback(self, future):
        try:
            result = future.result()
            print(result)
            if result.status == 4:  # STATUS_SUCCEEDED
                self.get_logger().info("목표 지점에 도착했습니다.")
            else:
                self.get_logger().warn(f"내비게이션 실패: 상태 {result.status}")
        except Exception as e:
            self.get_logger().error(f"내비게이션 중 오류 발생: {e}")
        finally:
            self.navigation_in_progress = False
            self.nav_goal_handle = None
            self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
            self.send_to_next_waypoint()  # 다음 웨이포인트로 이동

    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_and_recognize_faces(cv_image)

    def detect_and_recognize_faces(self, frame):
        res = self.vip.SIFT_feature_matching(frame)
        if res:
            self.get_logger().info("외부인 발견")
            self.stop_and_call_security_robot()

    def stop_and_call_security_robot(self):
        if self.security_robot_called:
            self.get_logger().info("이미 경비 로봇을 호출했습니다.")
            return

        self.get_logger().info("관제탑에 경비 로봇 호출 요청 중...")

        request = FindTarget.Request()
        point = Point()
        point.x = 100.0
        point.y = 100.0

        target = Target()
        target.object_id = 1
        target.class_name = "Intruder"
        target.object_position = point
        request.objects = [target,]
        request.patrol_id = self.patrol_id

        future = self.call_security_client.call_async(request)
        future.add_done_callback(self.handle_call_security_response)

    def handle_call_security_response(self, future):
        try:
            response: FindTarget.Response = future.result()
            if response.operation_successful:
                self.get_logger().info("경비 로봇 호출 성공!")
                if response.keep_track:
                    self.get_logger().info("정지 및 감시!")
                    self.security_robot_called = True
                    self.stopped_due_to_intruder = True
                    self.cancel_goal()
                else:
                    self.get_logger().info("계속 patrol모드")

            else:
                self.get_logger().warn("경비 로봇 호출 실패!")
        except Exception as e:
            self.get_logger().error(f"경비 로봇 호출 중 오류 발생: {e}")

    def security_arrival_callback(self, msg):
        if msg.data:
            self.get_logger().info("경비 로봇 도착! 순찰을 재개합니다.")
            self.security_robot_called = False
            self.stopped_due_to_intruder = False
            self.send_to_next_waypoint()
    
    def patrol_start(self):
        if self.security_robot_called or self.stopped_due_to_intruder:
            self.get_logger().info("순찰을 재개합니다.")
            self.security_robot_called = False
            self.stopped_due_to_intruder = False
            self.send_to_next_waypoint()

    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info("Robot stopped")

    def cancel_goal(self):
        self.security_robot_called = True
        self.stopped_due_to_intruder = True
        # 현재 목표를 취소
        if self.nav_goal_handle:
            self.nav_goal_handle.cancel_goal_async()
            self.get_logger().info("Goal cancelled")
            self.stop_robot()

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
