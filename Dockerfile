
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    git \
    tmux \
    ros-humble-xacro \
    ros-humble-tf2-ros \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-ur-description \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-ros-move-group \
    ros-humble-moveit-planners-ompl \
    ros-humble-moveit-kinematics \
    ros-humble-moveit-simple-controller-manager \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --ignore-installed \
    mediapipe==0.10.9 \
    opencv-python \
    "numpy>=1.24,<2.0"

RUN mkdir -p /opt/robotiq_ws/src && \
    cd /opt/robotiq_ws/src && \
    git clone https://github.com/PickNikRobotics/ros2_robotiq_gripper.git --depth 1 && \
    find /opt/robotiq_ws/src/ros2_robotiq_gripper -mindepth 1 -maxdepth 1 -type d \
        ! -name "robotiq_description" -exec rm -rf {} +

RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    cd /opt/robotiq_ws && \
    colcon build --symlink-install \
        --packages-select robotiq_description \
        --cmake-args -DCMAKE_BUILD_TYPE=Release"


COPY src/ /gesture_bot_ws/src/

RUN /bin/bash -c "\
    source /opt/ros/humble/setup.bash && \
    source /opt/robotiq_ws/install/setup.bash && \
    cd /gesture_bot_ws && \
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --packages-ignore robotiq_controllers robotiq_driver robotiq_hardware_tests"

RUN echo "source /opt/ros/humble/setup.bash"          >> /root/.bashrc && \
    echo "source /opt/robotiq_ws/install/setup.bash"  >> /root/.bashrc && \
    echo "source /gesture_bot_ws/install/setup.bash"  >> /root/.bashrc && \
    echo "export ROS_DOMAIN_ID=0"                      >> /root/.bashrc

WORKDIR /gesture_bot_ws

CMD ["/bin/bash", "-c", "\
    tmux new-session -d -s robot && \
    tmux send-keys -t robot '\
        source /opt/ros/humble/setup.bash && \
        source /opt/robotiq_ws/install/setup.bash && \
        source /gesture_bot_ws/install/setup.bash && \
        ros2 launch dual_arm_moveit_config verify_brain.launch.py' Enter && \
    tmux split-window -t robot -h && \
    tmux send-keys -t robot '\
        source /opt/ros/humble/setup.bash && \
        source /opt/robotiq_ws/install/setup.bash && \
        source /gesture_bot_ws/install/setup.bash && \
        sleep 6 && ros2 run gesture_teleop ik_commander' Enter && \
    tmux split-window -t robot -v && \
    tmux send-keys -t robot '\
        source /opt/ros/humble/setup.bash && \
        source /opt/robotiq_ws/install/setup.bash && \
        source /gesture_bot_ws/install/setup.bash && \
        sleep 12 && ros2 run gesture_teleop gesture_control_node' Enter && \
    tmux select-pane -t robot:0.0 && \
    tmux attach -t robot"]