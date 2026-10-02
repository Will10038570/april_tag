FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    nano \
    vim \
    python3.10 \
    python3.10-dev \
    python3.10-venv \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# Intel RealSense SDK (librealsense2) via the official RealSense apt repo
RUN apt-get update && apt-get install -y \
        gnupg2 \
        lsb-release \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /etc/apt/keyrings \
    && curl -sSf https://librealsense.realsenseai.com/Debian/librealsenseai.asc \
        | gpg --dearmor | tee /etc/apt/keyrings/librealsenseai.gpg > /dev/null \
    && echo "deb [signed-by=/etc/apt/keyrings/librealsenseai.gpg] https://librealsense.realsenseai.com/Debian/apt-repo $(lsb_release -cs) main" \
        | tee /etc/apt/sources.list.d/librealsense.list \
    && apt-get update \
    && apt-get install -y \
        librealsense2-utils \
        librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# ROS2 driver for RealSense
RUN apt-get update && apt-get install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description \
    && rm -rf /var/lib/apt/lists/*

# Python RealSense API
RUN pip3 install --no-cache-dir pyrealsense2

# cv_bridge / OpenCV for publishing and viewing images (e.g. rqt_image_view)
RUN apt-get update && apt-get install -y \
        python3-opencv \
        ros-humble-cv-bridge \
        ros-humble-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# Cyclone DDS rmw implementation (selected via RMW_IMPLEMENTATION in docker-compose.yml)
RUN apt-get update && apt-get install -y \
        ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# AprilTag detector + LQR controller deps (apriltag package)
# pupil-apriltags has no prebuilt wheel for arm64, so pip builds it from source;
# its build backend (scikit-build) needs a newer pip/setuptools/packaging than
# the base image ships, or "Getting requirements to build wheel" fails with a
# packaging version conflict. Upgrading only `packaging` isn't enough (the
# isolated build env still resolves against the old setuptools), so the whole
# toolchain is upgraded here.
# numpy is then pinned <2: cv_bridge's compiled boost module is built against
# the numpy 1.x C-API and breaks under numpy 2.x, which pupil-apriltags would
# otherwise pull in as a fresh transitive dependency.
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel packaging \
    && pip3 install --no-cache-dir "numpy<2" scipy pupil-apriltags

WORKDIR /mnt/work_space

VOLUME /mnt/work_space

RUN { \
        echo "source /opt/ros/humble/setup.bash"; \
        echo 'if [ -f /mnt/work_space/install/setup.bash ]; then source /mnt/work_space/install/setup.bash; fi'; \
    } >> /root/.bashrc

COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
