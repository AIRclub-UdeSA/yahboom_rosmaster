# syntax=docker/dockerfile:1
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# Common utilities, Gazebo Fortress apt repo, and ROS 2 Fortress packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg \
    lsb-release \
    sudo \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    xvfb \
    xauth \
    x11-apps \
    x11-utils \
    mesa-utils \
    && curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    ignition-fortress \
    ros-humble-ros-gz \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    ros-humble-tf-transformations \
    ros-humble-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Non-root user setup with matching host UID/GID. Reuse an existing numeric
# group when the host GID already exists in the base image.
ARG USERNAME=alumno
ARG USER_UID=1000
ARG USER_GID=1000

RUN if ! getent group "${USER_GID}" >/dev/null; then \
      groupadd --gid "${USER_GID}" "${USERNAME}"; \
    fi \
    && if ! getent group render >/dev/null; then groupadd --system render; fi \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && for group in dialout video render plugdev; do \
      if getent group "${group}" >/dev/null; then usermod -aG "${group}" ${USERNAME}; fi; \
    done \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

USER ${USERNAME}
WORKDIR /home/${USERNAME}/rosmaster_ws

# Setup ROS environment in user bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc \
    && echo "if [ -f /home/${USERNAME}/rosmaster_ws/install/setup.bash ]; then source /home/${USERNAME}/rosmaster_ws/install/setup.bash; fi" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
