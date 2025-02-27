
FROM ros:humble-ros-core-jammy

LABEL project='colav_gateway'
LABEL maintainer='Ryan McKee <r.mckee@qub.ac.uk>'
LABEL version='0.0.1'
LABEL description='ROS-based container for the colav_gateway application, providing a UDP-ROS bridge and managing control flow within the colav_gateway namespace.'

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
git \
openssh-client \
&& rm -rf /var/lib/apt/lists/*

# Install pip dependencies for colav_gateway
RUN pip install colav-protobuf-utils==0.0.2

# Clone colav_interfaces into colav_gateway container
# COPY .ssh /root/.ssh
# RUN chmod 700 /root/.ssh && chmod 600 /root/.ssh/* && ssh-keyscan github.com >> /root/.ssh/known_hosts
RUN mkdir -p /home/ros2_ws/src
RUN cd /home/ros2_ws/src && git clone https://github.com/RyanMcKeeQUB/colav_interfaces.git

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

COPY bootstrap_colav_gateway.sh /home/bootstrap_colav_gateway.sh
COPY colav_gateway /home/ros2_ws/src/colav_gateway
RUN chmod +x /home/bootstrap_colav_gateway.sh
ENTRYPOINT [ "/home/bootstrap_colav_gateway.sh" ]
