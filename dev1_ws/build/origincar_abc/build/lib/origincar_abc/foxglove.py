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
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Vector3
from simple_pid import PID
import math
from sensor_msgs.msg import Image
import cv2
from ai_msgs.msg import PerceptionTargets
from ai_msgs.msg import Target
from ai_msgs.msg import Point as aiPoint
from geometry_msgs.msg import Point32
from enum import Enum
import time


# 状态机
class State(Enum):
    STOP = 1, # 停车
    MOVE2QRCODE = 2, # 巡线，避障，扫码
    BACK_CAR = 3, # 扫到二维码后倒车
    MOVE2P = 4 # 往回走停车
    S_BZL = 5 # 向左边避障，开始
    E_BZL = 6 # 向左边避障，中
    jieshu_BZL = 7 # 向左边避障，结束
    BZR = 8 # 向右边避障
    TEST = 9 # 写避障用的
    # 冲两秒后减速转弯
    A1_LOW = 10


class Foxglove(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("开始订阅上位机中的话题...")
        self.pid_z = 0.0
        self.pid_speed = 3.0
        self.start_sign = self.create_timer(0.05, self.start_sign)
        self.loop = self.create_timer(0.05, self.loop_callback)
  

        
         
        # self.pid = PID(0.01, 0.006, 0.001, setpoint=1) # speed=1.0 nbnb
        # self.pid = PID(0.01, 0.006, 0.002, setpoint=1)# speed=0.8 nbnb
        # self.pid = PID(0.015, 0.006, 0.002, setpoint=0)# speed=0.8 nbnb
        # self.pid = PID(0.001, 0.0, 0.0, setpoint=1)# speed=0.8 nbnb
        # self.pid = PID(0.004, 0.0001, 0.0002, setpoint=0)# speed=0.8 nbnb


        # 状态机初始状态
        self.flag = State.STOP
        # 订阅上位机按键
        self.sub_sign4return = self.create_subscription(Int32, "sign4return", self.callback_sign4return, 1)
        # 订阅扫码结果
        self.sub_qrcode_info = self.create_subscription(String, "qrcode_detected/info_result", self.callback_qrcode_info, 1)
        # 发布阿克曼模型速度
        self.pub_ackermann_vel = self.create_publisher(AckermannDriveStamped, "ackermann_cmd", 1)
        # 订阅巡线速度
        # self.sub_line_cmd = self.create_subscription(Twist, "line_cmd_vel", self.callback_line_cmd, 1)
        # 订阅巡线的offset
        self.offset_sub = self.create_subscription(Point, "line_offset", self.callback_offset, 1)
         
        # 订阅避障发布的ai数据
        self.sub_ai_msg = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.callback_ai_msg, 10)
        
        # 全局阿克曼速度
        self.global_ackermann_vel = AckermannDriveStamped()
        self.cb_offset_msg = AckermannDriveStamped()

        # ================================================================================
        # self.pid = PID(0.01, 0.006, 0.001, setpoint=0)# speed=1.0 nbnb
        # self.pid = PID(0.01, 0.006, 0.002, setpoint=1)# speed=0.8 nbnb
        # self.pid = PID(0.015, 0.006, 0.002, setpoint=1)# speed=0.8 nbnb
        # self.pid = PID(0.01, 0.01, 0.002, setpoint=1)# speed=0.8 nb
        # ================================================================================

        self.pub_sign_switch = self.create_publisher(Sign, "/sign_switch", 1)
        init_qrcode_msg = Sign()
        init_qrcode_msg.sign_data = 1
        self.pub_sign_switch.publish(init_qrcode_msg)

    # ai_msgs.msg.Target(
    #     type='tong', 
    #     track_id=0, 
    #     rois=[
    #         ai_msgs.msg.Roi(
    #             type='tong', 
    #             rect=sensor_msgs.msg.RegionOfInterest(x_offset=305, y_offset=172, ai_height=39, ai_width=34, do_rectify=False), 
    #                 confidence=0.824922502040863)], 
    #         attributes=[], 
    #         points=[], 
    #         captures=[]
    # ),


    # 冲下面
    def a1_callback(self):
        self.flag = State.STOP
        self.flag = State.A1_LOW
        self.A1_LOW_startime = time.time()
        self.a1.cancel()


    def start_sign(self):
        input('回车后切换状态')
        self.a1 = self.create_timer(2, self.a1_callback)
        self.flag = State.MOVE2QRCODE
        self.start_sign.cancel()

    def callback_ai_msg(self, msg):
        # self.get_logger().info("msg.data: %d" % len(msg.targets))

        # 声明所需的tong的变量
        # ai_tong_cx = 0
        # ai_tong_cy = 0
        # ai_tong_height = 0
        # ai_tong_width = 0
        ai_tong_area = 0
        # 声明所需的line的变量
        ai_line_is_exist = False

        for ai_msg_n in msg.targets:
            if ai_msg_n.type == "tong":
                n_ai_height = ai_msg_n.rois[0].rect.height
                n_ai_width = ai_msg_n.rois[0].rect.width
                n_ai_area = n_ai_height * n_ai_width
                # n_ai_msg_cx = ai_msg_n.rois[0].rect.x_offset + n_ai_width / 2.0
                # n_ai_msg_cy = ai_msg_n.rois[0].rect.y_offset + n_ai_height / 2.0
                # 对比最大的
                ai_tong_area = n_ai_area if n_ai_area > ai_tong_area else ai_tong_area
            if ai_msg_n.type == "line":
                ai_line_is_exist = True

        # tong
        if ai_tong_area != 0:
            self.get_logger().info("area: %6d" % ai_tong_area)
            if 100000 >= ai_tong_area >= 6500:
                self.flag = State.S_BZL
                # 记录进入该状态的时间
                self.flag_S_BZL_start_time = time.time()
            if ai_tong_area > 100000:
                self.flag = State.STOP
                # 记录进入该状态的时间
                self.flag_S_BZL_start_time = time.time()

            ai_height = msg.targets[0].rois[0].rect.height
            ai_width = msg.targets[0].rois[0].rect.width
            ai_type = msg.targets[0].rois[0].type
            ai_msg_cx = msg.targets[0].rois[0].rect.x_offset + ai_width/2.0
            ai_msg_cy = msg.targets[0].rois[0].rect.y_offset + ai_height/2.0

            ai_roi0_area = ai_width * ai_height

            
            # self.get_logger().info("type: %4s, cxy: (%3d,%3d), area: %6d" % (ai_type, ai_msg_cx, ai_msg_cy, ai_roi0_area))
            # self.get_logger().info("area: %6d" % ai_tong_area)
            # if ai_type == "tong" and ai_tong_area <= 2500 and self.flag != State.S_BZL:
            #     self.flag = State.S_BZL
            #     # 记录进入该状态的时间
            #     self.flag_S_BZL_start_time = time.time()

        # max_x = 0
        # for ai_msg in msg.targets:
        #     ai_type = ai_msg.rois[0].type
        #     ai_msg_cx = ai_msg.rois[0].rect.x_offset
        #     ai_msg_cy = ai_msg.rois[0].rect.y_offset
        #     ai_height = ai_msg.rois[0].rect.ai_height
        #     ai_width = ai_msg.rois[0].rect.ai_width
        #     if ai_type == "tong" and ai_msg_cy <= 10:
        #         self.global_ackermann_vel.drive.speed = 1.0
        #         self.global_ackermann_vel.drive.steering_angle = -(math.pi / 2.0)
        #         # if ai_msg_cx >= max_x:
        #             # max_x = ai_msg_cx
        #         self.get_logger().info("ai_msg_cx: %d, ai_msg_cy: %d, ai_height: %d, ai_width: %d" % (ai_msg_cx, ai_msg_cy, ai_height, ai_width))
        # # self.get_logger().info("max_x: %d" % max_x)
        # self.get_logger().info("-----------------------------------------------")


    def callback_offset(self, msg):
        pass
        # # self.pid.reset()
        # self.pid_z = 0.0
        # # 计算PID转角
        # if self.flag == State.MOVE2QRCODE or self.flag == State.MOVE2P:
        #     # self.pid = PID(0.01, 0.006, 0.001, setpoint=0)
        #     self.pid_z = self.pid(msg.x-320)
        #     # 给左转加点增益
        #     # if self.pid_z < 0:
        #     #     self.pid_z *= 1.5


    # def callback_line_cmd(self, msg):
    def loop_callback(self):
        self.get_logger().info("状态机：%d" % self.flag.value)
        if self.flag == State.STOP:
            self.global_ackermann_vel.drive.speed = 0.0
            self.global_ackermann_vel.drive.steering_angle = 0.0
        # 任务一：巡线，避障，扫码
        if self.flag == State.MOVE2QRCODE:
            self.global_ackermann_vel.drive.speed = self.pid_speed
            self.global_ackermann_vel.drive.steering_angle = self.pid_z
        # 识别到二维码后倒车
        if self.flag == State.BACK_CAR:
            self.global_ackermann_vel.drive.speed = -1.0
            self.global_ackermann_vel.drive.steering_angle = 3.0
            # 倒车1.5秒钟后停止
            # self.BACK_CAR_timer = self.create_timer(1.5, self.BACK_CAR_timer_callback)
        # 手摇结束，巡线，避障
        if self.flag == State.MOVE2P:
            self.global_ackermann_vel.drive.speed = self.pid_speed
            self.global_ackermann_vel.drive.steering_angle = self.pid_z
        # 避障初始速度
        if self.flag == State.TEST:
            self.global_ackermann_vel.drive.speed = 1.0
            self.global_ackermann_vel.drive.steering_angle = 0.0
        # 从左侧避障, 开始
        if self.flag == State.S_BZL:
            self.global_ackermann_vel.drive.speed = 0.5
            # self.global_ackermann_vel.drive.steering_angle = 3.0
            self.global_ackermann_vel.drive.steering_angle = 3.0
            
            if time.time() - self.flag_S_BZL_start_time >= 0.2:
                # 记录进入该状态的时间
                self.flag_E_BZL_start_time = time.time()
                self.flag = State.E_BZL
        # 从左侧避障中
        if self.flag == State.E_BZL:
            self.global_ackermann_vel.drive.speed = 1.0
            self.global_ackermann_vel.drive.steering_angle = -3.0

            if time.time() - self.flag_E_BZL_start_time >= 0.3:
                self.flag_jieshu_BZL_start_time = time.time()
                self.flag = State.jieshu_BZL
        # 结束左边避障，回到线上
        if self.flag == State.jieshu_BZL:
            self.global_ackermann_vel.drive.speed = self.pid_speed
            self.global_ackermann_vel.drive.steering_angle = -3.0
            if time.time() - self.flag_jieshu_BZL_start_time >= 1:
                self.flag = State.STOP

        if self.flag == State.A1_LOW:
            self.global_ackermann_vel.drive.speed = 0.2
            self.global_ackermann_vel.drive.steering_angle = 3.0
            if time.time() - self.A1_LOW_startime >= 0.3:
                self.global_ackermann_vel.drive.speed = 0.2
                self.global_ackermann_vel.drive.steering_angle = 0.0
        # 从右侧避障
        # if self.flag == State.BZR:
        #     self.global_ackermann_vel.drive.speed = 1.0
        #     self.global_ackermann_vel.drive.steering_angle = math.pi / 2.0

        # 发布最终速度
        # self.get_logger().info("最终速度: x=%5.2f, z=%5.2f" % (self.global_ackermann_vel.drive.speed, self.global_ackermann_vel.drive.steering_angle))
        self.pub_ackermann_vel.publish(self.global_ackermann_vel)
        


    def BACK_CAR_timer_callback(self):
        self.flag = State.STOP
        self.get_logger().info("倒车结束, 状态变为STOP")
        self.BACK_CAR_timer.cancel()

    def S_BZL_timer_callback(self):
        self.flag = State.STOP
        # self.flag = State.E_BZL
        self.S_BZL_timer.cancel()









        

    #================================================================================================================
    # 上位机四个按键的回调函数 
    def callback_sign4return(self, msg):
        if msg.data == 0:
            self.get_logger().info("收到话题/sign4return发来的消息: 信号连接正常")
            self.flag = State.MOVE2QRCODE
            self.get_logger().info("状态变为: %d" % self.flag.value)

        elif msg.data == 5:
            self.get_logger().info("收到话题/sign4return发来的消息: 正在C区进行遥测")
            self.flag = State.STOP
            self.get_logger().info("状态变为: %d" % self.flag.value)

        elif msg.data == 6:
            self.get_logger().info("收到话题/sign4return发来的消息: C区出口结束遥测")
            self.flag = State.MOVE2P
            self.get_logger().info("状态变为: %d" % self.flag.value)

        elif msg.data == -1:
            self.get_logger().info("收到话题/sign4return发来的消息: 重置里程计")

    # 二维码识别的回调函数
    def callback_qrcode_info(self, msg):
        self.get_logger().info("扫描二维码得到的结果: %s" % msg.data)
        self.get_logger().info("===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n===============================================================================\n")
        qr_msg = Sign()
        if msg.data == "ClockWise":
            qr_msg.sign_data = 3
        if msg.data == "AntiClockWise":
            qr_msg.sign_data = 4
        self.pub_sign_switch.publish(qr_msg)

        # 识别到之后停车并倒车
        self.flag = State.STOP
        self.flag = State.BACK_CAR

    
def main(args=None):
    rclpy.init(args=args)
    foxglove = Foxglove("foxglove_node")
    rclpy.spin(foxglove)
    rclpy.shutdown()
