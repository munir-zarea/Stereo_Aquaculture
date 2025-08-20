from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('mmlove_stereo_recorder'),
        'config', 'sbs_camera.yaml'
    )
    with open(cfg, 'r') as f:
        p = yaml.safe_load(f)['sbs_camera']['ros__parameters']

    node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='stereo',
        name='sbs_cam',
        parameters=[{
            'video_device':    str(p.get('video_device', '/dev/video4')),
            'image_size':      list(p.get('image_size', [640, 480])),
            'time_per_frame':  list(p.get('time_per_frame', [1, 30])),
            'pixel_format':    str(p.get('pixel_format', 'YUYV')),
            'camera_frame_id': str(p.get('camera_frame_id', 'stereo_sbs_frame')),
        }],
        # Topic name is '/stereo/image_raw'. Use to find in rqt viewer
        remappings=[('image_raw', 'image_raw')],
    )
    return LaunchDescription([node])
