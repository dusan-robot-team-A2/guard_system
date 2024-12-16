from .central_node import CentralNode

from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseWithCovarianceStamped
from guard_interfaces.srv import FindTarget

class Patrol:
    _id_counter = 0
    def __init__(self,node:CentralNode, namespace = "gundam"):
        Patrol._id_counter += 1
        self.node = node
        self.patrol_id = Patrol._id_counter
        self.pose = (0,0)
        self.status = 1 #0: track, 1: patrol
        
        self.find_target_service = self.node.create_service(FindTarget, '/{namespace}/find_target', self.find_target_callback, 10)
        self.patrol_toggle = self.node.create_client(SetBool, f'/{namespace}/patrol_mode')
        # self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, f'/{namespace}/amcl_pose', self.pose_sub_callback, 10)
        self.pose_sub = self.node.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.pose_sub_callback, 10)

    def resume_patrol(self):
        request = SetBool.Request()
        request.data = True
        future = self.patrol_toggle.call_async(request)
        future.add_done_callback(self.resume_patrol_callback)
    
    def resume_patrol_callback(self,future):
        response = future.result()
        if response:
            self.node.get_logger().info('turn on patrol mode')
        else:
            self.node.get_logger().info('patrol mode service failed')

    def pose_sub_callback(self, msg:PoseWithCovarianceStamped):
        self.pose = msg.pose.pose.position.x, msg.pose.pose.position.y

    def find_target_callback(self, request:FindTarget.Request, response:FindTarget.Response):
        return self.find_target_callback(self.patrol_id, request, response)
    