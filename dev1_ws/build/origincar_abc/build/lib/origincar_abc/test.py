import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_msgs.msg import String
from origincar_msg.msg import Sign
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Float64
import time
from ackermann_msgs.msg._ackermann_drive_stamped import AckermannDriveStamped
# from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Vector3
from simple_pid import PID
import math

from sensor_msgs.msg import Image
import cv2

from ai_msgs.msg import PerceptionTargets
from ai_msgs.msg import Target
from ai_msgs.msg import Point as aiPoint
from geometry_msgs.msg import Point32



class Sub(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("开始订阅/sign4return")
        # self.sub_sign4return = self.create_subscription(Int32, "sign4return", self.callback_sign4return, 1)
        # self.sub_qrcode_info = self.create_subscription(String, "qrcode_detected/info_result", self.callback_qrcode_info, 1)

        # 发布阿克曼模型速度
        self.pub_ackermann_vel = self.create_publisher(AckermannDriveStamped, "ackermann_cmd", 1)
        self.global_ackermann_vel = AckermannDriveStamped()

        # 巡线
        self.sub_line_cmd = self.create_subscription(Twist, "line_cmd_vel", self.callback_line_cmd, 1)
        self.pub_pid_res = self.create_publisher(Vector3, "pid", 1)# 用于查看plot折线图（PID调试的时候用）
        # 有yolo
        self.pid = PID(0.007, 0.0, 0.002, setpoint=1)# speed=1.0 nbnb
        # self.pid = PID(0.007, 0.0, 0.001, setpoint=1)# speed=1.0 nbnb

        # 没有yolo
        # self.pid = PID(0.01, 0.006, 0.001, setpoint=0)# speed=1.0 nbnb
        self.pid.err_map = self.pid_err_limit
        # self.pid = PID(0.01, 0.006, 0.002, setpoint=1)# speed=0.8 nbnb
        # self.pid = PID(0.015, 0.006, 0.002, setpoint=1)# speed=0.8 nbnb
        # self.pid = PID(0.01, 0.01, 0.002, setpoint=1)# speed=0.8 nb
        mypid = Vector3()
        mypid.y = -1.0
        self.pub_pid_res.publish(mypid) # 用于重置plot折线图
        
        # 订阅图像
        self.sub_image = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.pub_image = self.create_publisher(Image, '/image_offset', 1)
        self.offset_cb_cv = 0
        
        # 发布AI消息
        self.pub_ai = self.create_publisher(PerceptionTargets, '/hobot_dnn_detection', 1)
        self.ai_msg = PerceptionTargets()
        self.ai_msg.fps = 60
        self.ai_msg.targets = [Target()]
        self.ai_msg.targets[0].type = 'object'
        self.ai_msg.targets[0].points = [aiPoint()]
        # self.ai_msg.targets[0].points[0].point = Point()
        # self.ai_msg.targets[0].points[0].point.x = 50.0
        # self.ai_msg.targets[0].points[0].point.y = 50.0
        
        
        # 是否开启二维码识别
        self.qrcode_enable = True

        # offset
        self.offset_sub = self.create_subscription(Point, "line_offset", self.callback_offset, 1)
        self.cb_offset_msg = AckermannDriveStamped()

        self.pub_sign_switch = self.create_publisher(Sign, "/sign_switch", 1)
        init_qrcode_msg = Sign()
        init_qrcode_msg.sign_data = 1
        self.pub_sign_switch.publish(init_qrcode_msg)

    def pid_err_limit(self, angle):
        # if abs(angle) > math.pi/2:
            # angle = 0
            
        return angle
    
    def image_callback(self, base_frame):
        # base_h, base_w, base_d = base_frame.shape
        frame = cv2.circle(base_frame, (int(self.offset_cb_cv.x), int(self.offset_cb_cv.y)), 3, (0, 0, 255), -1)
        
        self.pub_image.publish(frame)

        
    def callback_offset(self, msg):
        self.pid.reset()
        self.pid_z = 0.0
        self.ai_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_ai.publish(self.ai_msg)
        
        # 显示在图像
        self.offset_cb_cv = msg
        # 计算PID转角
        pid_z = self.pid(msg.x-320)
        # 给左转加点增益
        if pid_z<0:
            pid_z*=1.5
        # 全局初始速度
        self.global_ackermann_vel.drive.speed = 1.0
        self.global_ackermann_vel.drive.steering_angle = pid_z
        
        mypid = Vector3()
        mypid.x = pid_z
        self.pub_pid_res.publish(mypid)
        
        # 发布最终速度
        self.get_logger().info(
            "最终速度：x=%5.2f, z=%5.2f"%(
                self.global_ackermann_vel.drive.speed,
                self.global_ackermann_vel.drive.steering_angle))
        self.pub_ackermann_vel.publish(self.global_ackermann_vel)
        


    def callback_line_cmd(self, msg):
        pass
        # # 全局初始速度
        # self.global_ackermann_vel.drive.speed = 0.0
        # self.global_ackermann_vel.drive.steering_angle = 0.0
        
        # # 发布最终速度
        # self.get_logger().info(
        #     "最终速度：x=%5.2f, z=%5.2f"%(
        #         self.global_ackermann_vel.drive.speed,
        #         self.global_ackermann_vel.drive.steering_angle))
        # self.pub_ackermann_vel.publish(self.global_ackermann_vel)

        
    
def main(args=None):
    rclpy.init(args=args)
    sub = Sub("sub_sign4return")
    rclpy.spin(sub)
    rclpy.shutdown()