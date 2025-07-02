# Base image
FROM ros:galactic

# Setup environment
ENV DEBIAN_FRONTEND=noninteractive
ENV EUFS_MASTER=/workspace


# Install GUI tools
RUN apt update && apt install -y \
    libyaml-cpp-dev \
    x11-apps \
    ros-galactic-rviz2 \
    ros-galactic-gazebo-ros-pkgs \
    ros-galactic-rqt \
    ros-galactic-rqt-graph \
    ros-galactic-xacro \
    ros-galactic-ackermann-msgs \
    ros-galactic-image-geometry \
    python3-colcon-common-extensions \
    python3-pip \
    nano \
    python3-tk \
    python3-rosdep \
    wget \
    python3-rosdep \
    ros-galactic-joint-state-publisher \
    ros-galactic-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep (assumes root image)
RUN rosdep init || echo "rosdep already initialized"
RUN rosdep update

# Go back to workspace root
WORKDIR /workspace

ENTRYPOINT ["/bin/bash", "entrypoint.sh"]
