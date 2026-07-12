# Prebuilt image: ROS 2 Jazzy + Gazebo Harmonic + Nav2 + robot_localization.
# Build with:  docker build -t outdoor-sim:jazzy .
FROM osrf/ros:jazzy-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release ca-certificates \
        gdal-bin python3-gdal python3-pip \
        vim less git \
        ros-jazzy-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

# Python deps for the in-tree robo_fleet MCP server (./robo_fleet/).
# Installed globally so `python3 -u index.py` works inside the container.
COPY robo_fleet/requirements.txt /tmp/robo_fleet_requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages \
        -r /tmp/robo_fleet_requirements.txt \
    && rm /tmp/robo_fleet_requirements.txt

RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
        ros-jazzy-ros-gz \
        ros-jazzy-ros-gz-bridge \
        ros-jazzy-ros-gz-sim \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-robot-localization \
        ros-jazzy-slam-toolbox \
        ros-jazzy-xacro \
        ros-jazzy-foxglove-bridge \
        ros-jazzy-rosbridge-server \
    && rm -rf /var/lib/apt/lists/*

ENV LIBGL_ALWAYS_SOFTWARE=1 \
    GZ_HEADLESS_RENDERING=1 \
    QT_QPA_PLATFORM=offscreen

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

WORKDIR /workspace
CMD ["bash"]
