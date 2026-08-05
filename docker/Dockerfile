FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
      curl gnupg lsb-release ca-certificates \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
       -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
       > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
      ros-jazzy-ros-gz \
      ros-jazzy-demo-nodes-cpp \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-slam-toolbox \
      ros-jazzy-robot-state-publisher \
      ros-jazzy-joint-state-publisher \
      ros-jazzy-xacro \
      ros-jazzy-foxglove-bridge \
      ros-jazzy-rosbag2-storage-mcap \
      ros-jazzy-tf2-tools \
      ros-jazzy-launch-testing \
      ros-jazzy-launch-testing-ros \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
      python3-pip python3-colcon-common-extensions \
      libgl1-mesa-dri mesa-utils-extra \
      vim less tmux \
    && rm -rf /var/lib/apt/lists/* 

RUN pip3 install --break-system-packages \
      pytest pytest-html numpy pandas matplotlib pyyaml

ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe
ENV EGL_PLATFORM=surfaceless
ENV XDG_RUNTIME_DIR=/tmp/runtime
RUN mkdir -p /tmp/runtime && chmod 700 /tmp/runtime
ENV ROS_LOCALHOST_ONLY=1

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc \
 && echo "[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash" >> /root/.bashrc

WORKDIR /ws
CMD ["/bin/bash"]