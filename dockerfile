FROM ros:humble-ros-core-jammy

LABEL project='colav_gateway'
LABEL maintainer='Ryan McKee <r.mckee@qub.ac.uk>'
LABEL version='0.0.1'
LABEL description='ROS-based container for the colav_gateway application, providing a UDP-ROS bridge and managing control flow within the colav_gateway namespace.'

ARG MODE=container
ENV MODE=${MODE}

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
build-essential \
git \
python3-colcon-common-extensions \
python3-colcon-mixin \
python3-rosdep \
python3-vcstool \
&& rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
colcon mixin update && \
colcon metadata add default \
https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
ros-humble-ros-base=0.10.0-1* \
ros-humble-ament-cmake \
pip \
&& rm -rf /var/lib/apt/lists/*

# Install pip dependencies for colav_gateway
RUN pip install colav-protobuf-utils==0.0.2

RUN mkdir -p /home/ros2_ws/src

# Create colav_gateway directory and copy files if MODE is container
RUN if [ "$MODE" = "container" ]; then \
        mkdir -p /home/ros2_ws/src/colav_gateway && \
        cp -r ./colav_gateway /home/ros2_ws/src/colav_gateway; \
    fi
    
# Create colcon pkg and move colav_gateway to src
RUN cd /home/ros2_ws/src && \
    git clone https://github.com/RyanMcKeeQUB/colav_interfaces.git || { echo "git clone failed"; exit 1; }

# Build ros pkgs in ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /home/ros2_ws && \
    colcon build && \
    source ./install/setup.bash"
    
# Expose ports required for colav_gateway
# Mission Request
EXPOSE 7000/udp 
# Mission Response
EXPOSE 7001/udp
# Agent configuration
EXPOSE 7100/udp
# Obstacles configuration
EXPOSE 7200/udp
# Controller feedback
EXPOSE 7300/udp

# Source ros2_ws and ros2 install/setup.bash
RUN /bin/bash -c "echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && echo 'source /home/ros2_ws/install/setup.bash' >> /root/.bashrc"
RUN /bin/bash -c "source /root/.bashrc"

ENTRYPOINT [ "/bin/bash", "-c", "if [ \"$MODE\" = \"container\" ]; then source /opt/ros/humble/setup.bash && source /home/ros2_ws/install/setup.bash && ros2 launch colav_gateway colav_gateway.launch.py; else while true; do sleep 30; done" ]