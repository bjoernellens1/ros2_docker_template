ARG ROS_DISTRO=humble
ARG UNDERLAY_WS=/base_ws
ARG OVERLAY_WS=/overlay_ws

########################################
# Custom ROS2 Base Image               #
########################################

FROM osrf/ros:${ROS_DISTRO}-desktop as base
#ENV ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"] # change shell to bash because of better compatibility (standard shell would be sh otherwise).

ENV UNDERLAY_WS=${UNDERLAY_WS}
ENV OVERLAY_WS=${OVERLAY_WS}

# First checks
RUN echo "Underlay WS:" && $UNDERLAY_WS
RUN echo "Overlay WS:" && $OVERLAY_WS

# Create Colcon workspace with external dependencies
RUN mkdir -p ${UNDERLAY_WS}/src
WORKDIR ${UNDERLAY_WS}/src
COPY my.repos .
RUN mv my.repos dependencies.repos
RUN vcs import < dependencies.repos

# Build the base Colcon workspace, installing dependencies first.
WORKDIR ${UNDERLAY_WS}
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && apt-get update -y \
  && apt-get install -y --no-install-recommends \
   ros-${ROS_DISTRO}-turtlesim \
   ros-${ROS_DISTRO}-rqt \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --symlink-install

#ENV UNDERLAY_WS=${UNDERLAY_WS}

# Set up the entrypoint
COPY ./docker/my_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Update repository at the end of this build stage
RUN vcs pull src

ENTRYPOINT [ "/entrypoint.sh" ]


###########################################
# Overlay Image                           #
###########################################
FROM base AS overlay

# Create an overlay Colcon workspace
RUN mkdir -p ${OVERLAY_WS}/src
WORKDIR ${OVERLAY_WS}/src
COPY my_extended.repos ./overlay.repos
RUN vcs import < overlay.repos

WORKDIR ${OVERLAY_WS}

ENV DEBIAN_FRONTEND noninteractive 

RUN source ${UNDERLAY_WS}/install/setup.bash \
  && apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install \
    nano \
  && rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y \
  && rm -rf /var/lib/apt/lists/*

RUN source ${UNDERLAY_WS}/install/setup.bash \
  && colcon build --symlink-install

# Set up the entrypoint
COPY ./docker/my_entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Update repository at the end of this build stage
RUN vcs pull src

ENTRYPOINT [ "/entrypoint.sh" ]
