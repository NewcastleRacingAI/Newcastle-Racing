FROM osrf/ros:galactic-desktop


# Install dependencies from apt
RUN apt update && \ 
    apt install -y \
    libyaml-cpp-dev \
    python3-rosdep \
    x11-apps \
    ros-galactic-rviz2 \
    ros-galactic-rqt \
    ros-galactic-xacro \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-ackermann-msgs \
    ros-galactic-joint-state-publisher \
    ros-galactic-xacro \
    ros-galactic-rosbridge-server \
    ros-galactic-image-geometry \
    ros-galactic-joint-state-publisher \
    python3-colcon-common-extensions \
    python3-pip \
    python3-tk \
    nano \
    python3-rosdep \
    wget \
    can-utils


ENV NUFAI_MASTER=/workspace

WORKDIR ${NUFAI_MASTER}

# Build with colcon
RUN . /opt/ros/galactic/setup.sh && colcon build

# Setup .bashrc
RUN echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc && \ 
    echo 'source /workspace/install/setup.bash' >> ~/.bashrc

# THIS PART WILL BE UNCOMMENTED WHEN WE HAVE FINALIZED THE PACKAGE
#COPY newcastle_racing_ai newcastle_racing_ai
#COPY newcastle_racing_ai_msgs newcastle_racing_ai_msgs

# Build our package with colcon
#RUN . /opt/ros/galactic/setup.sh && colcon build --packages-select newcastle_racing_ai

ENTRYPOINT ["/bin/bash", "newcastle_racing_ai/entrypoint.sh"]
