# rosimg2mp4
A simple tool to record ROS2 Image topics to MP4. https://github.com/PINTO0309/simple-ros2-processing-tools

[![Downloads](https://static.pepy.tech/personalized-badge/rosimg2mp4?period=total&units=none&left_color=grey&right_color=brightgreen&left_text=Downloads)](https://pepy.tech/project/rosimg2mp4)

## 1. Install ROS2
```bash
DISTRO=humble

sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y \
ros-${DISTRO}-rosbag2 \
ros-${DISTRO}-vision-opencv \
ros-${DISTRO}-vision-msgs \
ros-${DISTRO}-image-pipeline
```
## 2. Install rosimg2mp4
```
pip install rosimg2mp4
```
## 3. Usage
```
usage: rosimg2mp4 [-h]
[-i RGB_IMAGE_TOPIC_NAME]
[-o OUTPUT_MP4_FILE_NAME]
[-fs FRAME_SIZE FRAME_SIZE]
[-vf VIDEO_WRITER_FPS]

options:
  -h, --help
    show this help message and exit
  -i RGB_IMAGE_TOPIC_NAME, --rgb_image_topic_name RGB_IMAGE_TOPIC_NAME
    RGB image topic name.
  -o OUTPUT_MP4_FILE_NAME, --output_mp4_file_name OUTPUT_MP4_FILE_NAME
    Output MP4 file name. e.g. output.mp4
  -fs FRAME_SIZE FRAME_SIZE, --frame_size FRAME_SIZE FRAME_SIZE
    Frame size. e.g. --frame-size {Width} {Height}
  -vf VIDEO_WRITER_FPS, --video_writer_fps VIDEO_WRITER_FPS
    Video writer FPS.
```

https://github.com/PINTO0309/mp42ros
