import sys
import rclpy
import threading
from rclpy.node import Node
from sensor_msgs.msg import Image
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from PySide2.QtGui import QPixmap, QImage
from geometry_msgs.msg import PoseWithCovarianceStamped

import yaml
from PIL import Image, ImageDraw

class NODE(QThread, Node):
    signal = Signal(list)

    def __init__(self, node_name='ros_subscriber_node'):
        QThread.__init__(self)
        Node.__init__(self, node_name)


        self.amcl_pose_x = 0
        self.amcl_pose_y = 0

        self.dot_size = 2

        self.sub_cmd_vel = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.subscription_callback, 10)



    def subscription_callback(self, msg):
        self.amcl_pose_x = msg.pose.pose.position.x
        self.amcl_pose_y = msg.pose.pose.position.y
        self.get_logger().info(f'Received amcl_pose x: {self.amcl_pose_x}, y: {self.amcl_pose_y}')

        self.signal.emit([self.amcl_pose_x, self.amcl_pose_y])
        self.get_logger().info(f'emit signal')


    def run(self):
        rclpy.spin(self)



class GUI(QMainWindow):
    def __init__(self, ros_thread):
        super().__init__()
        self.ros_thread = ros_thread
        self.ros_thread.signal.connect(self.process_signal)

        self.resize = (300,300)
        self.dot_size=2

        image = Image.open('src/example_service/map/map.pgm')
        self.width, self.height = image.size
        self.image_rgb = image.convert('RGB')

        with open('src/example_service/map/map.yaml', 'r') as file:

            data = yaml.safe_load(file)


        self.resolution = data['resolution']
        self.map_x= -data['origin'][0]
        self.map_y= +data['origin'][1] +self.height *self.resolution



        self.x = self.map_x
        self.y = self.map_y


        self.setupUi()


    def setupUi(self):
        self.window = QMainWindow()
        if not self.window.objectName():
            self.window.setObjectName(u"MainWindow")


        self.window.setObjectName("MainWindow")
        self.window.resize(340, 350)

        
        self.centralwidget = QWidget(self.window)
        self.centralwidget.setObjectName(u"centralwidget")


        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 20, 320, 320))

        self.window.setCentralWidget(self.centralwidget)



    def process_signal(self, message):
        odom_x = message[0]
        odom_y = message[1]
        self.x = self.map_x +odom_x
        self.y = self.map_y - odom_y

        print(f"x={self.x}, y={self.y}")
        self.input_image()



    def input_image(self):
        image_copy = self.image_rgb.copy()
        draw = ImageDraw.Draw(image_copy)
        draw.ellipse((

                self.x /self.resolution -self.dot_size, 
                self.y /self.resolution -self.dot_size, 
                self.x /self.resolution +self.dot_size, 
                self.y /self.resolution +self.dot_size), 
            fill='red')



        image_rotated = image_copy.rotate(90, expand=True)
        image_resized = image_rotated.resize(self.resize)
        pil_image = image_resized.convert('RGBA')  # QImage는 RGBA 포맷을 사용
        data = pil_image.tobytes("raw", "RGBA")
        qimage = QImage(data, *self.resize, QImage.Format_RGBA8888)
        pixmap = QPixmap.fromImage(qimage)
        self.label.setPixmap(pixmap)



def main():
    rclpy.init()
    node = NODE()
    ros_thread = threading.Thread(target=lambda : rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.window.show()

    
    try:
        sys.exit(app.exec_())
    finally:
        node.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()



    