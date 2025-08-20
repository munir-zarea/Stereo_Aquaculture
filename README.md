## Stereo_Aquaculture
This project implements a stereo video recording and processing system using a stereo camera integrated with ROS 2. The goal is to reliably capture synchronized left and right video streams and store them in a reproducible format.


Prerequisites
# Core viewer/tools. Enter into terminal
```bash
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-v4l2-camera \
                    ros-${ROS_DISTRO}-rqt-image-view \
                    ros-${ROS_DISTRO}-image-view \
                    ffmpeg
''bash

1) Identify your camera device
v4l2-ctl --list-devices

Note the SBS node for your “3D USB Camera” (e.g., /dev/video4).
Tip: prefer stable symlinks:

ls -l /dev/v4l/by-id/
# Example: /dev/v4l/by-id/usb-3D_USB_Camera_...-index0 -> ../../video4

2) Configure the package

Edit the params file:

nano ~/stereo_ws/src/mmlove_stereo_recorder/config/sbs_camera.yaml


Set your device path (use /dev/v4l/by-id/...-index0 if available):

sbs_camera:
  ros__parameters:
    video_device: "/dev/video4"
    image_size: [640, 480]     # raise later (e.g., [1280, 720])
    time_per_frame: [1, 30]    # 30 FPS (numerator, denominator)
    pixel_format: "YUYV"       # or "MJPG" if your camera supports it
    camera_frame_id: "stereo_sbs_frame"

3) Build & source
cd ~/stereo_ws
colcon build --symlink-install
source ~/stereo_ws/install/setup.bash

4) Launch the single SBS stream
ros2 launch mmlove_stereo_recorder sbs.launch.py


Verify:

ros2 topic list | grep /stereo/image_raw
ros2 topic hz /stereo/image_raw

5) View the video

GUI viewer:

ros2 run rqt_image_view rqt_image_view
# select /stereo/image_raw


CLI viewer (lightweight):

ros2 run image_view image_view --ros-args -r image:=/stereo/image_raw

6) Record to a bag (recommended settings)
ros2 bag record -o ~/stereo_ws/bags/sbs_run \
  /stereo/image_raw \
  --storage mcap --compression-mode file --compression-format zstd \
  --max-bag-size 4096


Stop with Ctrl+C. Inspect:

ros2 bag info ~/stereo_ws/bags/sbs_run

7) Replay later
ros2 bag play ~/stereo_ws/bags/sbs_run

8) Export a single MP4 (optional)

Save frames while playing (or from the live stream):

ros2 run image_view image_saver \
  --ros-args -r image:=/stereo/image_raw \
  -p save_all_image:=true \
  -p filename_format:=/tmp/sbs_%06d.png


Find the true FPS from the bag (ros2 bag info: frames / duration), then encode:

ffmpeg -framerate 30 -i /tmp/sbs_%06d.png -c:v libx264 -pix_fmt yuv420p stereo_sbs.mp4


(If you publish/record /stereo/image_raw/compressed, use .jpg and add -p image_transport:=compressed to image_saver.)
