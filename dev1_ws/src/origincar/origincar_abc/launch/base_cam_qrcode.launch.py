from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()
    
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/root/dev1_ws/src/origincar/origincar_base/launch/origincar_bringup.launch.py')
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/root/dev1_ws/src/origincar/origincar_bringup/launch/usb.launch.py')
    )

    qrcode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/root/dev1_ws/src/origincar/originbot_qr_code/qr_code_detection/launch/qr_code_detection.launch.py')
    )

    ld.add_action(base_launch)
    ld.add_action(camera_launch)
    ld.add_action(qrcode_launch)

    return ld