# ROS2_BA

sudo apt install ros-jazzy-eigen-stl-containers
rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y


colcon build
