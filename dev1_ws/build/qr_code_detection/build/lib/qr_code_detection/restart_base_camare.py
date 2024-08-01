import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Sub(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("关闭节点启动")
        self.sub = self.create_subscription(String, "/qrcode_detected/info_result", self.sub_callback,10)

    def sub_callback(self, msg):
        self.get_logger().info("识别到二维码信息：%s" % msg.data)
        # self.pub = self.create_publisher(String, "destroy_qrcode", 10)
        # msg = String()
        # msg.data = "1"
        # self.pub.publish(msg)
        # self.get_logger().info("收到二维码信息后发送销毁qrcode_detect节点话题: /destroy_qrcode")



def main(args=None):
    rclpy.init(args=args)
    close_node = Sub("close_node")
    rclpy.spin(close_node)
    rclpy.shutdown()