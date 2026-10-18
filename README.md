# ROS2_BA

Dependencies installation for new machines:
    sudo apt install ros-jazzy-eigen-stl-containers
    rosdep update --rosdistro=$ROS_DISTRO
    sudo apt-get update
    rosdep install --from-paths src --ignore-src -r -y

Launching simulation environement:
    cd ROS2_BA/
    ./run.sh 

Adding obstacles:
    cd ROS2_BA/src/scenes/
    python3 scene_loader.py stage_a
