ARG ROS_DISTRO=humble
ARG UNDERLAY_WS=/rmp_ws

########################################
# Custom ROS2 Base Image               #
########################################

FROM osrf/ros:${ROS_DISTRO}-desktop as base
#ENV ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"] # change shell to bash because of better compatibility (standard shell would be sh otherwise).

# Create Colcon workspace with external dependencies
RUN mkdir -p /${UNDERLAY_WS}/src
WORKDIR /${UNDERLAY_WS}/src
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
