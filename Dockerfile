# Base image
FROM ros:humble

# Setup environment
ENV DEBIAN_FRONTEND=noninteractive
ENV EUFS_MASTER=/workspace

# Install GUI tools
RUN apt update && apt install -y \
    libyaml-cpp-dev \
    x11-apps \
    python3-colcon-common-extensions \
    python3-pip \
    python3-tk \
    python3-rosdep \
    nano \
    vim \
    wget

# Initialize rosdep
RUN rosdep update
RUN rosdep init || echo "rosdep already initialized"

# Create workspace root
WORKDIR /workspace

# Copy the ros packages (the ones we are not supposed to be actively developing)
COPY src/eufs_msgs ./src/eufs_msgs
COPY src/ros_can ./src/ros_can
COPY src/zed-ros2-wrapper ./src/zed-ros2-wrapper
COPY src/ft-fsd-path-planning ./src/ft-fsd-path-planning
COPY src/eufs_rviz_plugins ./src/eufs_rviz_plugins
# Install dependencies (Round 1)
RUN rosdep install --from-paths src --ignore-src -r -y

# Copy the eufs_sim package (the one we are actively developing)
COPY src/eufs_sim ./src/eufs_sim
# Install dependencies (Round 2)
RUN rosdep install --from-paths src --ignore-src -r -y

# Copy the rest of the source code
COPY src ./src
# Install dependencies (Final Round)
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-skip zed_components zed_ros2 zed_wrapper

COPY launch_simulator.sh /workspace/launch_simulator.sh

# Add user called user for people developing inside the container, in the case
# that a user only has one user account on their system they will both have UID 1000
# and the host machine will see files created in the container as files created by 
# the Host systems user. 
RUN useradd -m -G users,sudo -s /usr/bin/bash user
# Remove passwd for users
RUN passwd -d root
RUN passwd -d user
# If your host machine UID is not 1000 change the container 
# user's UID with the following command
# RUN usermod -u 1001

RUN chmod +x /workspace/launch_simulator.sh
RUN echo "source /workspace/install/setup.bash\nexport PS1=\"(nrai docker) \$PS1\"" >> /root/.bashrc
RUN echo "source /workspace/install/setup.bash\nexport PS1=\"(nrai docker) \$PS1\"" >> /home/user/.bashrc

# Run personal configureaiton steps
COPY ./personal_config.sh /root/personal_config.sh
RUN /root/personal_config.sh

CMD ["./launch_simulator.sh"]
