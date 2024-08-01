#include "origincar_base/origincar_base.h"
#include "rclcpp/rclcpp.hpp"
#include "origincar_base/Quaternion_Solution.h"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp" 
#include "origincar_msg/msg/data.hpp"
#include "origincar_msg/msg/sign.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"


enum class State{
    STOP, //停车
    MOVE2QRCODE, // 巡线，避障，扫码
    BACK_CAR, // 扫到二维码后倒车
    MOVE2P // 往回走停车
};

static State flag = State::STOP;

class Foxglove: public rclcpp::Node
{
private:
    // 订阅上位机的四个按键
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int32>> sub_sign4return;
    // 发布上位机信号
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> pub_sign_foxglove;
    // 订阅二维码消息
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> sub_qrcode;
    // 发布二维码信息
    std::shared_ptr<rclcpp::Publisher<origincar_msg::msg::Sign>> pub_qrcode;

    // 发布阿克曼速度
    std::shared_ptr<rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>> pub_ackermann_vel;
    // 订阅巡线节点发送的数据
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Twist>> sub_line_cmd_vel;
    // 订阅巡线节点的offset
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::Point>> sub_offset;

    ackermann_msgs::msg::AckermannDriveStamped cb_offset_msg;
    // 全局速度
    ackermann_msgs::msg::AckermannDriveStamped global_speed;

public:
    Foxglove(std::string name): Node(name)
    {
        RCLCPP_INFO(get_logger(), "创建订阅上位机信号节点...");
        sub_sign4return = this->create_subscription<std_msgs::msg::Int32>("sign4return", 1, 
            [this](const std_msgs::msg::Int32::SharedPtr msg)
            {
                if(msg->data == 0)
                    RCLCPP_INFO(get_logger(), "订阅到上位机的信号: 信号连接正常");
                    flag = State::MOVE2QRCODE;
                if(msg->data == 5)
                    RCLCPP_INFO(get_logger(), "订阅到上位机的信号: 正在C区进行遥测");
                    flag = State::STOP;
                if(msg->data == 6)
                    RCLCPP_INFO(get_logger(), "订阅到上位机的信号: C区出口结束遥测");
                    flag = State::MOVE2P;
                if(msg->data == -1)
                    RCLCPP_INFO(get_logger(), "订阅到上位机的信号: 重置里程计");

                std_msgs::msg::Int32 pub_sign_foxglove_msg;
                pub_sign_foxglove_msg.data = msg->data;
                pub_sign_foxglove->publish(pub_sign_foxglove_msg);
                RCLCPP_INFO(get_logger(), "发布上位机的信号: %d", pub_sign_foxglove_msg.data);
            });

        RCLCPP_INFO(get_logger(), "创建发布上位机信号节点...");
        pub_sign_foxglove = this->create_publisher<std_msgs::msg::Int32>("sign_foxglove", 1);


        pub_qrcode = this->create_publisher<origincar_msg::msg::Sign>("sign_switch", 1);
        // 初始化小车信号
        origincar_msg::msg::Sign qrcode_init_msg;
        qrcode_init_msg.sign_data = 1;
        pub_qrcode->publish(qrcode_init_msg);


        RCLCPP_INFO(get_logger(), "创建订阅二维码信息节点...");
        sub_qrcode = this->create_subscription<std_msgs::msg::String>("qrcode_detected/info_result", 1, 
            [this](const std_msgs::msg::String::SharedPtr msg)
            {
                origincar_msg::msg::Sign qrcode_msg;
                if(msg->data == "ClockWise") // 3
                {
                    qrcode_msg.sign_data = 3;
                    RCLCPP_INFO(get_logger(), "扫码信息: %s -> 3", msg->data.c_str());
                }
                else if(msg->data == "AntiClockWise") // 4
                {
                    qrcode_msg.sign_data = 4;
                    RCLCPP_INFO(get_logger(), "扫码信息: %s -> 4", msg->data.c_str());
                }
                // 发布二维码信息
                pub_qrcode->publish(qrcode_msg);
                // 进入回调函数说明，扫到二维码后先停车然后倒车
                flag = State::STOP;
                flag = State::BACK_CAR;
            });

        // 订阅巡线offset
		sub_offset = this->create_subscription<geometry_msgs::msg::Point>("line_offset", 1, 
            [this](const geometry_msgs::msg::Point::SharedPtr msg)
            {
                cb_offset_msg.drive.speed = 0.3;
                cb_offset_msg.drive.steering_angle = (320 - msg->x) * 0.004;
            });

        // 初始化阿克曼速度发布者
        pub_ackermann_vel = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("ackermann_cmd", 1);

        // 订阅巡线速度
        sub_line_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>( "line_cmd_vel", 5, 
            [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                if(flag == State::STOP)
                {
                    global_speed.drive.speed = 0.0;
                    global_speed.drive.steering_angle = 0.0;
                }

                if(flag == State::MOVE2QRCODE)
                {
                    RCLCPP_INFO(get_logger(), "任务一：巡线，避障，扫码");
                    global_speed.drive.speed = cb_offset_msg.drive.speed;
                    global_speed.drive.steering_angle = cb_offset_msg.drive.steering_angle;
                }

                if(flag == State::BACK_CAR)
                {
                    RCLCPP_INFO(get_logger(), "识别到二维码后倒车");
                    global_speed.drive.speed = -0.8;
                    global_speed.drive.steering_angle = M_PI / 2.0;
                }

                if(flag == State::MOVE2P)
                {
                    RCLCPP_INFO(get_logger(), "做完任务回停车点");
                    global_speed.drive.speed = cb_offset_msg.drive.speed;
                    global_speed.drive.steering_angle = cb_offset_msg.drive.steering_angle;
                }

                pub_ackermann_vel->publish(global_speed);

            });
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto foxglove  = std::make_shared<Foxglove>("foxglove_node");
    rclcpp::spin(foxglove);
    rclcpp::shutdown();
    return 0;
}