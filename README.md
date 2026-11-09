# ROS2_BA

Dependencies installation for new machines:
    sudo apt install ros-jazzy-eigen-stl-containers
    rosdep update --rosdistro=$ROS_DISTRO
    sudo apt-get update
    rosdep install --from-paths src --ignore-src -r -y
    
Add all subfolders to Python import:
    nano ~/.bashrc
    
    Add this:
        # Python: make src and every folder directly inside it importable
        for d in "$HOME/ROS2_BA/src"/*/; do
            d="${d%/}"
            case "$(basename "$d")" in __pycache__|.*) continue ;; esac
            export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$d"
        done
        export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$HOME/ROS2_BA/src"
        
    source ~/.bashrc

Launching simulation environement:
    cd ROS2_BA/
    ./run.sh

Adding obstacles:
    cd ROS2_BA/src/scenes/
    python3 scene_loader.py stage_a
