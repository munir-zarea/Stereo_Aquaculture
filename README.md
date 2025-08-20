# Stereo_Aquaculture
This project implements a stereo video recording and processing system using a stereo camera integrated with ROS 2. The goal is to reliably capture synchronized left and right video streams and store them in a reproducible format. Please note that this process is done using an MMLove Stereo USB camera. Other cameras still need to be tested to ensure the code is modular.



## 1) Prerequisites
a) Clone repository:
```bash
cd
mkdir -p ~/stereo_ws/src
cd ~/stereo_ws/src
```
Now you will be in the a dedicated workspace for the project.

b) Install dependencies:
Depending on the distro, your installation packages might look different. This one is Jazzy.
```bash
sudo apt update
sudo apt install ros-jazzy-v4l2-camera ros-jazzy-image-transport \
                 ros-jazzy-camera-info-manager ros-jazzy-compressed-image-transport \
                 ros-jazzy-cv-bridge ros-jazzy-image-view
```
c) Build the workspace:
From the root of the workspace (~/stereo_ws)
```bash
cd ~/stereo_ws
colcon build --symlink-install
```

d) After a successful build, source the workspace:
```bash
source install/setup.bash
```
e) Include core viewer/tools that must be installed. Enter into terminal
```bash
sudo apt update
sudo apt install -y ros-${ROS_DISTRO}-v4l2-camera \
                    ros-${ROS_DISTRO}-rqt-image-view \
                    ros-${ROS_DISTRO}-image-view \
                    ffmpeg
```

f) Now, go into edit each individual file created to look like the ones listed in this repository. Please contact zaream@hawaii.edu if any questions.

## 2) Identify your camera device
```bash
v4l2-ctl --list-devices
```

Note the SBS node for your camera (e.g., /dev/video4). It should be something like "3D USB Camera" or similar.

## 3) Configure the package

Edit the config file (sbs_camera.yaml). Set your device path (use /dev/v4l/by-id/...-index0 if available):

```bash
sbs_camera:
  ros__parameters:
    video_device: "/dev/video4" # Change this depending on you camera input
    image_size: [640, 480]     # Increase later
    time_per_frame: [1, 30]    # 30 fps
    pixel_format: "YUYV"       # or "MJPG" if your camera supports it
    camera_frame_id: "stereo_sbs_frame"
```

## 4) Build & source workspace
```bash
cd ~/stereo_ws
colcon build --symlink-install
source ~/stereo_ws/install/setup.bash
```

## 5) Launch the single SBS stream
```bash
ros2 launch mmlove_stereo_recorder sbs.launch.py
```

Verify the topics are publishing:
```bash
ros2 topic list | grep /stereo/image_raw
ros2 topic hz /stereo/image_raw
```

## 6) View the video

GUI viewer (run this in another terminal as the launch file is running):
```bash
ros2 run rqt_image_view rqt_image_view
```

Select '/stereo/image_raw' from the dropdown menu.

## 7) Record to a bag (recommended settings)

Open a third terminal and run:
```bash
ros2 bag record -o ~/stereo_ws/bags/sbs_run \
  /stereo/image_raw \
  --storage mcap --compression-mode file --compression-format zstd \
  --max-bag-size 4096
```

Stop with Ctrl+C and inspect:
```bash
ros2 bag info ~/stereo_ws/bags/sbs_run
```


