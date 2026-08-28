FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8
ENV ROS_DISTRO=humble
ENV USER_NAME=ros
ENV USER_UID=1000
ENV USER_GID=1000
ENV HOME=/home/ros
ENV DISPLAY=:1
ENV VNC_DISPLAY=:1
ENV VNC_PORT=5901
ENV VNC_GEOMETRY=1280x800
ENV VNC_DEPTH=24
ENV VNC_PASSWORD=ros
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    bash-completion \
    build-essential \
    ca-certificates \
    curl \
    dbus-x11 \
    git \
    gnupg \
    iproute2 \
    less \
    locales \
    lsb-release \
    mesa-utils \
    nano \
    net-tools \
    openssh-client \
    python3-pip \
    sudo \
    tigervnc-common \
    tigervnc-standalone-server \
    tigervnc-tools \
    tmux \
    wget \
    x11-xserver-utils \
    xauth \
    xfce4 \
    xfce4-goodies \
    xfonts-100dpi \
    xfonts-75dpi \
    xfonts-base \
    xterm \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
      libeigen3-dev \
      libyaml-cpp-dev \
      python3-colcon-common-extensions \
      python3-rosdep \
      python3-vcstool \
      ros-humble-gz-ros2-control \
      ros-humble-joint-state-broadcaster \
      ros-humble-robot-state-publisher \
      ros-humble-robot-localization \
      ros-humble-desktop \
      ros-dev-tools \
      ros-humble-ros-gz \
      ros-humble-ros-gz-bridge \
      ros-humble-ros-gz-image \
      ros-humble-ros-gz-interfaces \
      ros-humble-ros-gz-sim \
      ros-humble-ros2-control \
      ros-humble-ros2-controllers \
      ros-humble-std-msgs \
      ros-humble-teleop-twist-keyboard \
      ros-humble-tf2-geometry-msgs \
      ros-humble-xacro \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true

RUN groupadd --gid "${USER_GID}" "${USER_NAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash "${USER_NAME}" \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}" \
    && chmod 0440 "/etc/sudoers.d/${USER_NAME}" \
    && mkdir -p /workspace "${HOME}/.vnc" \
    && chown -R "${USER_NAME}:${USER_NAME}" /workspace "${HOME}"

COPY start-vnc.sh /usr/local/bin/start-vnc.sh
RUN sed -i 's/\r$//' /usr/local/bin/start-vnc.sh \
    && chmod +x /usr/local/bin/start-vnc.sh

USER ${USER_NAME}
WORKDIR /workspace

RUN rosdep update \
    && echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> "${HOME}/.bashrc" \
    && echo "if [ -f /workspace/install/setup.bash ]; then source /workspace/install/setup.bash; fi" >> "${HOME}/.bashrc" \
    && echo "export QT_X11_NO_MITSHM=1" >> "${HOME}/.bashrc" \
    && echo "export LIBGL_ALWAYS_SOFTWARE=1" >> "${HOME}/.bashrc"

EXPOSE 5901

CMD ["/usr/local/bin/start-vnc.sh"]
