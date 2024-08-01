// Copyright (c) 2022, www.guyuehome.com

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/point.hpp"


using hbm_img_msgs::msg::HbmMsg1080P;
using sensor_msgs::msg::CompressedImage;
using sensor_msgs::msg::Image;
using std::placeholders::_1;

class Nv122BGR : public rclcpp::Node
{
public:
    double offset_x,offset_y;

    Nv122BGR(std::string node_name = "Nv122BGR") : Node(node_name)
    {
        RCLCPP_INFO(this->get_logger(), node_name);

        // 创建消息订阅者，从摄像头节点订阅图像消息
        sublisher_ = this->create_subscription_hbmem<HbmMsg1080P>(
            "/hbmem_image", 1, std::bind(&Nv122BGR::image_callback, this, std::placeholders::_1));
        compress_pub_ = this->create_publisher<CompressedImage>("/image_out/compressed", 1);
        image_pub_ = this->create_publisher<Image>("/image_out", 1);

        // 
        offset_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/line_offset", 1, std::bind(&Nv122BGR::pointCallback, this, std::placeholders::_1));
        tiny_image_pub_ = this->create_publisher<Image>("/image_out_tiny", 1);
    }

private:

    void pointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // 处理接收到的点消息
        // RCLCPP_ERROR(this->get_logger(), "Received point: x=%f, y=%f, z=%f", msg->x, msg->y, msg->z);
        this->offset_x = msg->x;
        this->offset_y = msg->y;
    }


    void image_callback(const HbmMsg1080P::ConstSharedPtr img_msg)
    {
        // 对订阅到的图片消息进行验证，本示例只支持处理NV12格式图片数据
        if (!img_msg)
            return;
        if ("nv12" != std::string(reinterpret_cast<const char *>(img_msg->encoding.data())))
        {
            RCLCPP_ERROR(rclcpp::get_logger("Nv122BGR"), "Only support nv12 img encoding!");
            return;
        }
        cv::Mat nv12Image(img_msg->height * 3 / 2, img_msg->width, CV_8UC1,
                          const_cast<uint8_t *>(img_msg->data.data()));
        cv::Mat bgrImage;//640*480
        cv::cvtColor(nv12Image, bgrImage, cv::COLOR_YUV2BGR_NV12);
	    // RCLCPP_INFO( rclcpp::get_logger( "Image_Transport_Node" ), "Input Image width is %d, height is %d", width_, height_ );
        
        // // +缩小到0.5的分辨率-此处图片会输入qrcode
        // cv::Mat bgrImageSmall;
        // // +缩小前在图片上面画出识别到的line点
        cv::Point point((int)this->offset_x, (int)this->offset_y);
        cv::circle(bgrImage, point, 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        // cv::resize(bgrImage, bgrImageSmall, cv::Size(), 0.5, 0.5, cv::INTER_LANCZOS4);
        // // end

        // +此处第二个参数原为 bgrImage
        // cv::Point point(100, 100);
        // x-640, y-480
        // cv::Point point((int)this->offset_x, (int)this->offset_y);
        // cv::circle(bgrImage, point, 3, cv::Scalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        //画出line_follower的ROI-296行代码
        // cv::Rect rectangle(0, 240, 640-1, 240); // 定义矩形，参数依次为左上角的x坐标、y坐标，矩形的宽度和高度
        // cv::rectangle(bgrImage, rectangle, cv::Scalar(0, 255, 0), 2); // 在图像上绘制矩形，绿色，线宽为2

        // auto cvImage = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgrImage);
        // compress_pub_->publish(std::move(*cvImage.toCompressedImageMsg(cv_bridge::JPEG)));
        // image_pub_->publish(std::move(*cvImage.toImageMsg()));

        // +发布缩小到0.2倍分辨率的图片
        cv::Mat bgrImageTiny;
        cv::resize(bgrImage, bgrImageTiny, cv::Size(), 0.2, 0.2, cv::INTER_LANCZOS4);
        auto cvImageTiny = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", bgrImageTiny);
        tiny_image_pub_->publish(std::move(*cvImageTiny.toImageMsg()));
    }
    rclcpp::Publisher<CompressedImage>::SharedPtr compress_pub_;
    rclcpp::Publisher<Image>::SharedPtr image_pub_;
    rclcpp::SubscriptionHbmem<HbmMsg1080P>::ConstSharedPtr sublisher_ = nullptr;

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr offset_subscriber_;
    rclcpp::Publisher<Image>::SharedPtr tiny_image_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Nv122BGR>("Nv122BGR");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
