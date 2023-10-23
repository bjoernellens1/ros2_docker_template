ARG ROS_DISTRO=humble
ARG UNDERLAY_WS=/base_ws
ARG OVERLAY_WS=/overlay_ws

########################################
# Custom ROS2 Base Image               #
########################################

FROM osrf/ros:${ROS_DISTRO}-desktop as base
#ENV ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"] # change shell to bash because of better compatibility (standard shell would be sh otherwise).

# Create Colcon workspace with external dependencies
RUN mkdir -p /base_ws/src
WORKDIR /base_ws/src
COPY my.repos .
RUN mv my.repos dependencies.repos
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR /base_ws
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && apt-get update -y \
  && apt-get install -y --no-install-recommends \
   ros-${ROS_DISTRO}-turtlesim \
   ros-${ROS_DISTRO}-rqt \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/my_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Update repository at the end of this build stage
RUN vcs pull src

ENTRYPOINT [ "/entrypoint.sh" ]


#############################################################
# Overlay Image -- Building Unitree GO1 ROS2 implementation #
#############################################################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p /overlay_ws/src
WORKDIR /overlay_ws/src
COPY my_extended.repos ./overlay.repos
RUN vcs import < overlay.repos \
  && cd unitree_go1 \
  && git clone -b v3.5.1 https://github.com/unitreerobotics/unitree_legged_sdk \
  && mv ros2_unitree_legged_msgs/ ..
WORKDIR /lcm_ws
RUN cd /lcm_ws \
  #&& git clone -b v1.4.0 https://github.com/lcm-proj/lcm \
  && git clone https://github.com/lcm-proj/lcm \
  && cd lcm \
  && mkdir build \
  && cd build \
  && cmake ../ \
  && make \
  && make install

WORKDIR /overlay_ws

ENV DEBIAN_FRONTEND noninteractive 

RUN source /base_ws/install/setup.bash \
  && apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install \
    nano \
  #&& rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
  && rm -rf /var/lib/apt/lists/*

RUN source /base_ws/install/setup.bash \
  && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/my_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Update repository at the end of this build stage
RUN vcs pull src

ENTRYPOINT [ "/entrypoint.sh" ]
#CMD [ "ros2", "run", "unitree_legged_real", "ros2_udp highlevel" ]
CMD ldconfig -v; ros2 run unitree_legged_real ros2_udp highlevel
