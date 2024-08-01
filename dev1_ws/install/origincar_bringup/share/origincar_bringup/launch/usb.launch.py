import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import TextSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory, get_package_prefix

def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument('device', default_value='/dev/video8', description='usb camera device'),
        DeclareLaunchArgument('line_follower_model_path', default_value='/root/dev1_ws/src/origincar/originbot_deeplearning/line_follower_perception/model/resnet18_224x224_nv12_official.bin', description=' '),
        DeclareLaunchArgument('line_follower_model_name', default_value='resnet18_224x224_nv12', description=' '),
    ]

    # Include launch descriptions
    usb_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_usb_cam') + '/launch/hobot_usb_cam.launch.py'),
                                       launch_arguments={'usb_image_width': '640', 'usb_image_height': '480',
                                                         'usb_video_device': LaunchConfiguration('device')}.items())

    jpeg_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_encode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'shared_mem', 'codec_out_mode': 'ros',
                                                                 'codec_sub_topic': '/hbmem_img', 'codec_pub_topic': '/image'}.items())

    nv12_codec_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'ros', 'codec_out_mode': 'shared_mem',
                                                                 'codec_sub_topic': '/image', 'codec_pub_topic': '/hbmem_image'}.items())

    nv12_codec_node_yolo = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('hobot_codec') + '/launch/hobot_codec_decode.launch.py'),
                                               launch_arguments={'codec_in_mode': 'ros', 'codec_out_mode': 'shared_mem',
                                                                 'codec_sub_topic': '/image', 'codec_pub_topic': '/hbmem_img','output_framerate':'1'}.items())


    web_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(get_package_share_directory('websocket') + '/launch/websocket.launch.py'),
                                        launch_arguments={
                                            'websocket_image_topic': '/image', 
                                            'websocket_image_type': 'mjpeg',
                                            'websocket_only_show_image': 'True',
                                            }.items())
    # line_follower
    line_follower_node = Node(
        package='line_follower_perception',
        executable='line_follower_perception',
        output='screen',
        parameters=[
            {"model_path": LaunchConfiguration('line_follower_model_path')},
            {"model_name": LaunchConfiguration('line_follower_model_name')},
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription(launch_args + [
        usb_node, 
        # nv12_codec_node,
        nv12_codec_node_yolo,
        # jpeg_codec_node,
        line_follower_node,
        # web_node
    ])