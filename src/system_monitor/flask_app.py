from flask import Flask, Response, render_template, Request
from .sm_node import SystemMonitoringNode
from threading import Thread
from flask_cors import CORS
import cv2
import rclpy
import time
import json

rclpy.init()
smNode = SystemMonitoringNode()
app = Flask(__name__)
CORS(app)

# ROS 2 노드를 별도의 스레드에서 실행
def spin_ros2_node(node):
    rclpy.spin(node)
    rclpy.shutdown()
        
# def generate_frames_cctv():
#     while True:
#         # Read the camera frame
#         time.sleep(0.1)
#         frame = smNode.cctv_image_frame

#         if frame is not None:
#             # Encode the frame in JPEG format
#             ret, buffer = cv2.imencode('.jpg', frame)
#             frame = buffer.tobytes()
#             # Concatenate frame bytes with multipart data structure
#             yield (b'--frame\r\n'
#                     b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            
def generate_sm_variable():
    while True:
        time.sleep(0.1)

        data = smNode.system_info_str

        yield f"data: {data}\n\n"

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/data')
def data():
    return Response(generate_sm_variable(), content_type='text/event-stream')

# @app.route('/video_feed')
# def video_feed():
#     # Returns the video stream response
#     return Response(generate_frames_cctv(),
#                     mimetype='multipart/x-mixed-replace; boundary=frame')
    

if __name__ == "__main__":
    ros2_thread = Thread(target=spin_ros2_node, args=[smNode], daemon=True)
    ros2_thread.start()
    app.run(debug=True)