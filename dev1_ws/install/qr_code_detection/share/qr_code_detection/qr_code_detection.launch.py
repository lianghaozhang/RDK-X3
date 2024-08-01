#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2022, www.guyuehome.com

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#     http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    web_service_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('websocket'),
                'launch/websocket_service.launch.py'))
    )
    hobot_codec_node = Node(
        package='hobot_codec',
        executable='hobot_codec_republish',
        output='screen',
        parameters=[
                {"channel": 1},
                {"in_mode": "ros"},
                {"in_format": "bgr8"},
                {"sub_topic": "/qrcode_detected/img_result"},
                {"out_mode": "ros"},
                {"out_format": "jpeg"},
                {"pub_topic": "/image_jpeg"}
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    # 订阅：/image_jpeg
    websocket_node = Node(
        package='websocket',
        executable='websocket',
        output='screen',
        parameters=[
                {"image_topic": "/image_jpeg"},
                {"image_type": "mjpeg"},
                {"only_show_image": True},
                {"smart_topic": "/ai_msg_mono2d_trash_detection"},
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    image_transport_node = Node(
        package='utils',
        executable='image_transport_node',
        arguments=['--ros-args', '--log-level', 'info']
    )
    # 发布的话题：/qrcode_detected/img_result
    qr_detection_node = Node(
        package='qr_code_detection',
        executable='qr_detection_node',
    )
    
    return LaunchDescription([
        image_transport_node,
        qr_detection_node,
        # hobot_codec_node,
        # hobot_usb_cam_node,
        # web_service_launch_include,
        # websocket_node,
    ])
