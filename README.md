ROS2 HUMBLE 공식 홈페이지에서 설치

sudo apt-get update
sudo apt-get install libeigen3-dev

client package빌드할 때:
pip install --upgrade setuptools

sudo apt-get install ros-humble-tf-transformations

sudo pip3 install transforms3d
pip install numpy scipy


pip install open3d scikit-learn
pip uninstall empy
pip install empy==3.3.4
sudo apt install libvirt-dev
sudo apt install ros-humble-apriltag-msgs

sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-*
sudo apt install ros-humble-image-transport-plugins

rosdep install --from-paths src --ignore-src -r -y
pip install catkin_pkg
pip install lark

sudo apt install mesa-utils
sudo ubuntu-drivers autoinstall

sudo apt-get update && sudo apt-get install -y \
     ros-humble-joint-state-publisher-gui \
     ros-humble-xacro \
     ros-humble-ros2-control \
     ros-humble-moveit* \
     ros-humble-ros2-controllers \
     ros-humble-ros-gz-* \
     ros-humble-*-ros2-control
     
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-gazebo-ros2-control
