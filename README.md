# ROS2_BA

Dependencies installation for new machines:
    sudo apt install ros-jazzy-eigen-stl-containers
    rosdep update --rosdistro=$ROS_DISTRO
    sudo apt-get update
    rosdep install --from-paths src --ignore-src -r -y

Add all subfolders to Python import:
    nano ~/.bashrc

    #Add this:
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
    cd src/my_code/scenes/
    python3 scene_loader.py stage_...

OMPL planning:
    cd src/my_code/OMPL_planning/
    python3 plan_joint_space.py ...

Plotting OMPL planning:
    cd src/my_code/OMPL_planning/
    python3 plot_path.py path/<file>.json --deg

Creating B-splines:
    cd src/my_code/b_splines
    python3 bspline_traj.py path/<file>.json --dts 1.5 1.5 ...

Plotting B-splines:
    cd src/my_code/b_splines
    python3 plot_traj.py path<file>.json --deg

Test time function:
    cd src/my_code/Objective_Functions/
    python3 test_f1.py path<file>.json

Test jerk function:
    cd src/my_code/Objective_Functions/
    python3 test_f3.py path<file>.json
